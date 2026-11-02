/*
 * PPG bridge: BLE observer -> USB CDC ACM.
 *
 * - Scans continuously (never connects) for advertisements from a sensor
 *   named "BEACON" that carry PPG samples in manufacturer-specific data.
 * - Each advertisement is decoded into (red, ir, sensor time) samples.
 *   The same advertisement is heard many times; repeats are dropped by
 *   their sensor timestamp.
 * - Samples are written to the USB CDC ACM port as one JSON object per line:
 *       {"red":54123,"ir":78234,"t_ms":123456}\n
 *   but only while the host has DTR asserted.
 * - Lines received from the host are parsed; {"cmd":"sync"} is accepted for
 *   compatibility but only logged (there is no connection to read from).
 *   Buffered samples are flushed as soon as DTR is asserted.
 *
 * Connect from a PC with: tio /dev/ttyACM0
 */

#include <errno.h>
#include <stddef.h>
#include <string.h>

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/sys/atomic.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/ring_buffer.h>
#include <zephyr/data/json.h>
#include <zephyr/logging/log.h>

#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/gap.h>

#include "usb_dev.h"

LOG_MODULE_REGISTER(ppg_bridge, LOG_LEVEL_INF);

/* ======================================================================
 * Configuration
 * ====================================================================== */

#define BLE_PERIPHERAL_NAME   "BEACON" // TODO: change this its test
#define BLE_MIN_RSSI          (-70)   /* ignore weaker advertisements */

/*
 * ADVERTISEMENT FORMAT (legacy advertising, 31 bytes max):
 *
 *   Flags                         3 B
 *   Complete name "BEACON"        2 + 6 B
 *   Manufacturer specific data    2 B header (len, 0xFF), then:
 *     company ID  uint16 LE       PPG_COMPANY_ID
 *     version     uint8           PPG_ADV_VERSION
 *     records     N x 12 B        (one record fits in legacy advertising)
 *
 * The name must be in the advertisement itself, not the scan response,
 * because scanning is passive.
 */
#define PPG_COMPANY_ID        0xFFFFU /* SIG-reserved for internal/test use */
#define PPG_ADV_VERSION       0x01U
#define PPG_MFR_HDR_SIZE      3       /* company ID + version */

/*
 * ASSUMED BLE PAYLOAD FORMAT -- adapt decode_ble_payload() if yours differs.
 * The manufacturer data (after its header) carries N back-to-back records:
 *
 *   offset 0  uint32 LE  sensor timestamp, ms (peripheral clock)
 *   offset 4  uint32 LE  red ADC value (18-bit, 0..262143)
 *   offset 8  uint32 LE  ir  ADC value (18-bit, 0..262143)
 */
#define PPG_REC_SIZE          12
#define PPG_ADC_MAX           0x3FFFFU

/* If the peripheral clock jumps back by more than this, treat it as a
 * peripheral reboot and re-anchor; smaller steps back are duplicates. */
#define PPG_TS_REBASE_MS      1000U

#define PPG_SAMPLE_QUEUE_LEN  256     /* ~2.5 s at 100 Hz */
#define PPG_LINE_MAX          1024    /* host->device line limit */

/* ======================================================================
 * Sample queue (BLE -> USB)
 * ====================================================================== */

struct ppg_sample {
	uint64_t t_ms;   /* session-monotonic time, ms */
	uint32_t red;
	uint32_t ir;
};

K_MSGQ_DEFINE(sample_q, sizeof(struct ppg_sample), PPG_SAMPLE_QUEUE_LEN, 8);

static atomic_t dropped_samples;
static atomic_t matched_advs;   /* health counters, cleared every 10 s */
static atomic_t new_samples;

static void enqueue_sample(const struct ppg_sample *s)
{
	/* Drop the oldest sample when full so the stream stays current. */
	while (k_msgq_put(&sample_q, s, K_NO_WAIT) != 0) {
		struct ppg_sample old;

		(void)k_msgq_get(&sample_q, &old, K_NO_WAIT);
		atomic_inc(&dropped_samples);
	}
}

/* ----------------------------------------------------------------------
 * Timestamp mapping.
 *
 * t_ms must be nonnegative and monotonic for the whole firmware session,
 * even across a peripheral reboot. The sensor's own clock is kept (it carries
 * the precise sample spacing the pulse estimator wants), but shifted by an
 * offset anchored to this device's uptime. The offset is chosen on the first
 * sample and again whenever the sensor clock resets, so output never goes
 * backwards. Repeated copies of the same advertisement carry the same sensor
 * timestamp and are dropped here as duplicates.
 *
 * Touched only from the BT RX thread (scan callback).
 * ---------------------------------------------------------------------- */
static bool     ts_anchored;
static int64_t  ts_offset;
static uint32_t last_sensor_ts;
static uint64_t last_out_ts;
static bool     any_out_ts;

static bool map_timestamp(uint32_t sensor_ts, uint64_t *out)
{
	if (ts_anchored && sensor_ts <= last_sensor_ts) {
		if (last_sensor_ts - sensor_ts < PPG_TS_REBASE_MS) {
			return false;   /* duplicate (e.g. read after notify) */
		}
		ts_anchored = false;    /* peripheral clock reset */
	}

	if (!ts_anchored) {
		uint64_t base = (uint64_t)k_uptime_get();

		if (any_out_ts && base <= last_out_ts) {
			base = last_out_ts + 1;
		}
		ts_offset = (int64_t)base - (int64_t)sensor_ts;
		ts_anchored = true;
	}

	last_sensor_ts = sensor_ts;
	*out = (uint64_t)((int64_t)sensor_ts + ts_offset);
	if (any_out_ts && *out < last_out_ts) {
		*out = last_out_ts;     /* defensive; should not happen */
	}
	last_out_ts = *out;
	any_out_ts = true;
	return true;
}

static void decode_ble_payload(const uint8_t *data, uint16_t len)
{
	if (len % PPG_REC_SIZE) {
		LOG_WRN("payload %u B not a multiple of %u; trailing bytes ignored",
			len, PPG_REC_SIZE);
	}

	for (uint16_t off = 0; off + PPG_REC_SIZE <= len; off += PPG_REC_SIZE) {
		uint32_t ts  = sys_get_le32(&data[off + 0]);
		uint32_t red = sys_get_le32(&data[off + 4]);
		uint32_t ir  = sys_get_le32(&data[off + 8]);
		struct ppg_sample s;

		/* The phone rejects out-of-range values; never send them. */
		if (red > PPG_ADC_MAX || ir > PPG_ADC_MAX) {
			LOG_WRN("out-of-range sample red=%u ir=%u dropped", red, ir);
			continue;
		}
		if (!map_timestamp(ts, &s.t_ms)) {
			continue;
		}
		s.red = red;
		s.ir = ir;
		enqueue_sample(&s);
		atomic_inc(&new_samples);
	}
}

/* ======================================================================
 * USB CDC ACM
 * ====================================================================== */

static const struct device *const cdc_dev =
	DEVICE_DT_GET_ONE(zephyr_cdc_acm_uart);

RING_BUF_DECLARE(tx_rb, 2048);
RING_BUF_DECLARE(rx_rb, 512);
static struct k_spinlock tx_lock;
static struct k_spinlock rx_lock;

static atomic_t dtr_on;
static atomic_t rx_overflow;
static K_SEM_DEFINE(dtr_sem, 0, 1);

static void rx_work_handler(struct k_work *work);
static K_WORK_DEFINE(rx_work, rx_work_handler);

static void cdc_isr(const struct device *dev, void *user_data)
{
	ARG_UNUSED(user_data);

	/* uart_irq_update() returns void in this Zephyr version. */
	for (uart_irq_update(dev); uart_irq_is_pending(dev);
	     uart_irq_update(dev)) {
		if (uart_irq_rx_ready(dev)) {
			uint8_t buf[64];
			int n = uart_fifo_read(dev, buf, sizeof(buf));

			if (n > 0) {
				k_spinlock_key_t key = k_spin_lock(&rx_lock);
				uint32_t put = ring_buf_put(&rx_rb, buf, n);

				k_spin_unlock(&rx_lock, key);
				if (put < (uint32_t)n) {
					atomic_set(&rx_overflow, 1);
				}
				k_work_submit(&rx_work);
			}
		}

		if (uart_irq_tx_ready(dev)) {
			k_spinlock_key_t key = k_spin_lock(&tx_lock);
			uint8_t *p;
			uint32_t avail = ring_buf_get_claim(&tx_rb, &p, 64);

			if (avail == 0) {
				uart_irq_tx_disable(dev);
			} else {
				int sent = uart_fifo_fill(dev, p, avail);

				ring_buf_get_finish(&tx_rb, sent > 0 ? sent : 0);
			}
			k_spin_unlock(&tx_lock, key);
		}
	}
}

static void set_dtr(bool on)
{
	bool was = atomic_set(&dtr_on, on ? 1 : 0);

	if (on && !was) {
		LOG_INF("DTR asserted, streaming");
		/* Report carrier/DSR via the CDC notification endpoint. */
		(void)uart_line_ctrl_set(cdc_dev, UART_LINE_CTRL_DCD, 1);
		(void)uart_line_ctrl_set(cdc_dev, UART_LINE_CTRL_DSR, 1);
		k_sem_give(&dtr_sem);
	} else if (!on && was) {
		LOG_INF("DTR deasserted, buffering");
	}
}

static void usb_msg_cb(struct usbd_context *const ctx,
		       const struct usbd_msg *msg)
{
	ARG_UNUSED(ctx);

	switch (msg->type) {
	case USBD_MSG_CDC_ACM_CONTROL_LINE_STATE: {
		uint32_t dtr = 0U;

		(void)uart_line_ctrl_get(msg->dev, UART_LINE_CTRL_DTR, &dtr);
		set_dtr(dtr != 0U);
		break;
	}
	case USBD_MSG_CDC_ACM_LINE_CODING: {
		/* Accept whatever the host asks (phone sends 115200 8N1);
		 * baud rate is meaningless on native USB. */
		uint32_t baud = 0U;

		(void)uart_line_ctrl_get(msg->dev, UART_LINE_CTRL_BAUD_RATE, &baud);
		LOG_INF("line coding: %u baud", baud);
		break;
	}
	case USBD_MSG_RESET:
	case USBD_MSG_VBUS_REMOVED:
		set_dtr(false);
		break;
	default:
		break;
	}
}

/* Writes whole lines only, and only while DTR is asserted. */
static void usb_writer_thread(void *p1, void *p2, void *p3)
{
	ARG_UNUSED(p1);
	ARG_UNUSED(p2);
	ARG_UNUSED(p3);

	struct ppg_sample s;
	char line[80];

	for (;;) {
		if (!atomic_get(&dtr_on)) {
			/* Samples keep accumulating in sample_q meanwhile. */
			k_sem_take(&dtr_sem, K_FOREVER);
			continue;
		}

		if (k_msgq_get(&sample_q, &s, K_MSEC(100)) != 0) {
			continue;
		}

		int len = snprintk(line, sizeof(line),
				   "{\"red\":%u,\"ir\":%u,\"t_ms\":%llu}\n",
				   s.red, s.ir, (unsigned long long)s.t_ms);

		if (len <= 0 || len >= (int)sizeof(line)) {
			continue;
		}

		while (atomic_get(&dtr_on)) {
			k_spinlock_key_t key = k_spin_lock(&tx_lock);
			bool fits = ring_buf_space_get(&tx_rb) >= (uint32_t)len;

			if (fits) {
				ring_buf_put(&tx_rb, (uint8_t *)line, len);
			}
			k_spin_unlock(&tx_lock, key);

			if (fits) {
				uart_irq_tx_enable(cdc_dev);
				break;
			}
			k_msleep(1);   /* host is slow; wait for room */
		}
	}
}

K_THREAD_DEFINE(usb_writer, 1024, usb_writer_thread, NULL, NULL, NULL,
		7, 0, 0);

/* ======================================================================
 * BLE observer
 * ====================================================================== */

struct adv_info {
	bool name_ok;
	const uint8_t *mfr;   /* only valid while device_found() is running */
	uint8_t mfr_len;
};

/* Collects the name match and our manufacturer data in one pass. */
static bool parse_ad(struct bt_data *data, void *user_data)
{
	struct adv_info *info = user_data;

	switch (data->type) {
	case BT_DATA_NAME_COMPLETE:
	case BT_DATA_NAME_SHORTENED:
		if (data->data_len == strlen(BLE_PERIPHERAL_NAME) &&
		    memcmp(data->data, BLE_PERIPHERAL_NAME, data->data_len) == 0) {
			info->name_ok = true;
		}
		break;
	case BT_DATA_MANUFACTURER_DATA:
		if (data->data_len >= PPG_MFR_HDR_SIZE &&
		    sys_get_le16(data->data) == PPG_COMPANY_ID &&
		    data->data[2] == PPG_ADV_VERSION) {
			info->mfr = data->data + PPG_MFR_HDR_SIZE;
			info->mfr_len = data->data_len - PPG_MFR_HDR_SIZE;
		}
		break;
	default:
		break;
	}

	return true;   /* check every field */
}

/* Runs on the BT RX thread for every advertisement heard. */
static void device_found(const bt_addr_le_t *addr, int8_t rssi, uint8_t type,
			 struct net_buf_simple *ad)
{
	static bool seen;
	struct adv_info info = { 0 };

	if (type != BT_GAP_ADV_TYPE_ADV_NONCONN_IND &&
	    type != BT_GAP_ADV_TYPE_ADV_SCAN_IND &&
	    type != BT_GAP_ADV_TYPE_ADV_IND) {
		return;
	}

	bt_data_parse(ad, parse_ad, &info);
	if (!info.name_ok || !info.mfr) {
		return;
	}

	if (rssi < BLE_MIN_RSSI) {
		return;
	}

	if (!seen) {
		char addr_str[BT_ADDR_LE_STR_LEN];

		bt_addr_le_to_str(addr, addr_str, sizeof(addr_str));
		LOG_INF("Sensor found: %s (RSSI %d)", addr_str, rssi);
		seen = true;
	}

	atomic_inc(&matched_advs);

	/* Decode now: the advertisement buffer is not kept after return. */
	decode_ble_payload(info.mfr, info.mfr_len);
}

static int start_scan(void)
{
	/* Duplicate filtering off: many controllers filter by address only,
	 * which would hide every advertisement after the first. Window ==
	 * interval keeps the radio listening continuously. */
	int err = bt_le_scan_start(BT_LE_SCAN_PARAM(BT_LE_SCAN_TYPE_PASSIVE,
						    BT_LE_SCAN_OPT_NONE,
						    0x0060, 0x0060),
				   device_found);

	if (err) {
		LOG_ERR("Scanning failed to start (err %d)", err);
		return err;
	}
	LOG_INF("Scanning started");
	return 0;
}

/* ======================================================================
 * Host -> device commands
 * ====================================================================== */

struct host_cmd {
	char *cmd;
};

static const struct json_obj_descr host_cmd_descr[] = {
	JSON_OBJ_DESCR_PRIM(struct host_cmd, cmd, JSON_TOK_STRING),
};

static void handle_sync(void)
{
	/* No connection to read from in observer mode. Buffered samples are
	 * already flushed by the writer as soon as DTR is up. Kept so host
	 * software that sends sync keeps working. */
	LOG_INF("sync requested (no-op in advertising mode)");
}

static void handle_line(char *line, size_t len)
{
	struct host_cmd cmd = { 0 };

	/* Accept CRLF and trailing whitespace. */
	while (len && (line[len - 1] == '\r' || line[len - 1] == ' ' ||
		       line[len - 1] == '\t')) {
		line[--len] = '\0';
	}
	if (len == 0) {
		return;
	}

	int ret = json_obj_parse(line, len, host_cmd_descr,
				 ARRAY_SIZE(host_cmd_descr), &cmd);

	if (ret < 0 || !(ret & BIT(0)) || !cmd.cmd) {
		LOG_DBG("ignored host line");
		return;
	}

	if (strcmp(cmd.cmd, "sync") == 0) {
		handle_sync();
	} else {
		LOG_DBG("unknown cmd '%s'", cmd.cmd);
	}
}

/* Reassembles lines across fragmented USB packets; several lines per
 * packet are fine. Lines over PPG_LINE_MAX are dropped through the next
 * newline. Runs on the system workqueue. */
static void rx_work_handler(struct k_work *work)
{
	static char line[PPG_LINE_MAX + 1];
	static size_t line_len;
	static bool discarding;
	uint8_t buf[64];

	ARG_UNUSED(work);

	if (atomic_clear(&rx_overflow)) {
		/* Bytes were lost; the current line is corrupt. */
		discarding = true;
		line_len = 0;
	}

	for (;;) {
		k_spinlock_key_t key = k_spin_lock(&rx_lock);
		uint32_t n = ring_buf_get(&rx_rb, buf, sizeof(buf));

		k_spin_unlock(&rx_lock, key);
		if (n == 0) {
			break;
		}

		for (uint32_t i = 0; i < n; i++) {
			char c = (char)buf[i];

			if (c == '\n') {
				if (!discarding) {
					line[line_len] = '\0';
					handle_line(line, line_len);
				}
				discarding = false;
				line_len = 0;
				continue;
			}
			if (discarding) {
				continue;
			}
			if (line_len >= PPG_LINE_MAX) {
				LOG_WRN("host line > %d bytes discarded",
					PPG_LINE_MAX);
				discarding = true;
				line_len = 0;
				continue;
			}
			line[line_len++] = c;
		}
	}
}

/* ======================================================================
 * main
 * ====================================================================== */

int main(void)
{
	int err;

	if (!device_is_ready(cdc_dev)) {
		LOG_ERR("CDC ACM device not ready");
		return 0;
	}

	uart_irq_callback_set(cdc_dev, cdc_isr);

	err = ppg_usb_init(usb_msg_cb);
	if (err) {
		LOG_ERR("USB init failed (%d)", err);
		return 0;
	}

	uart_irq_rx_enable(cdc_dev);

	err = bt_enable(NULL);
	if (err) {
		LOG_ERR("Bluetooth init failed (err %d)", err);
		return 0;
	}
	LOG_INF("Bluetooth initialized");

	err = start_scan();
	if (err) {
		return 0;
	}

	/* Periodic health log on the debug console (never on the CDC port). */
	for (;;) {
		k_sleep(K_SECONDS(10));
		long a = atomic_clear(&matched_advs);
		long n = atomic_clear(&new_samples);
		long d = atomic_clear(&dropped_samples);

		LOG_INF("last 10 s: %ld matching advertisements, %ld new samples",
			a, n);
		if (d) {
			LOG_WRN("%ld samples dropped (queue full)", d);
		}
	}
	return 0;
}