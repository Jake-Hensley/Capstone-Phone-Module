/*
 * PPG bridge: BLE central -> USB CDC ACM.
 *
 * - Scans for a peripheral named "Hake", subscribes to its PPG characteristic.
 * - Each notification is decoded into (red, ir, sensor time) samples.
 * - Samples are written to the USB CDC ACM port as one JSON object per line:
 *       {"red":54123,"ir":78234,"t_ms":123456}\n
 *   but only while the host has DTR asserted.
 * - Lines received from the host are parsed; {"cmd":"sync"} triggers a fresh
 *   GATT read of the PPG characteristic and flushes any buffered samples.
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
#include <zephyr/bluetooth/hci.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/bluetooth/gatt.h>

#include "usb_dev.h"

LOG_MODULE_REGISTER(ppg_bridge, LOG_LEVEL_INF);

/* ======================================================================
 * Configuration
 * ====================================================================== */

#define BLE_PERIPHERAL_NAME   "Hake" // TODO: change this its test
#define BLE_MIN_RSSI          (-20)   /* connect only to close devices */

/* Replace with the peripheral's real (ideally 128-bit custom) UUIDs. */
static struct bt_uuid_16 hake_svc_uuid = BT_UUID_INIT_16(0x180D);
static struct bt_uuid_16 hake_chr_uuid = BT_UUID_INIT_16(0x2A38);

/*
 * ASSUMED BLE PAYLOAD FORMAT -- adapt decode_ble_payload() if yours differs.
 * A notification (or read response) carries N back-to-back records:
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
 * even across BLE reconnects or a peripheral reboot. The sensor's own clock
 * is kept (it carries the precise sample spacing the pulse estimator wants),
 * but shifted by an offset anchored to this device's uptime. A new offset is
 * chosen on each connection, and whenever the sensor clock resets, so output
 * never goes backwards.
 *
 * Touched only from the BT RX thread (connected/notify/read callbacks).
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

	while (uart_irq_update(dev) && uart_irq_is_pending(dev)) {
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
 * BLE central
 * ====================================================================== */

static struct k_spinlock conn_lock;
static struct bt_conn *default_conn;
static atomic_t chr_ready;          /* subscribed & value handle valid */
static uint16_t chr_value_handle;

static struct bt_gatt_discover_params discover_params;
static struct bt_gatt_subscribe_params subscribe_params;
static struct bt_gatt_exchange_params mtu_params;
static struct bt_gatt_read_params read_params;
static atomic_t read_busy;

static void start_scan(void);

static void clear_default_conn(void)
{
	k_spinlock_key_t key = k_spin_lock(&conn_lock);
	struct bt_conn *c = default_conn;

	default_conn = NULL;
	k_spin_unlock(&conn_lock, key);

	if (c) {
		bt_conn_unref(c);
	}
}

static uint8_t notify_func(struct bt_conn *conn,
			   struct bt_gatt_subscribe_params *params,
			   const void *data, uint16_t length)
{
	ARG_UNUSED(conn);

	if (!data) {
		LOG_INF("Unsubscribed");
		params->value_handle = 0U;
		atomic_clear(&chr_ready);
		return BT_GATT_ITER_STOP;
	}

	decode_ble_payload(data, length);
	return BT_GATT_ITER_CONTINUE;
}

static uint8_t discover_func(struct bt_conn *conn,
			     const struct bt_gatt_attr *attr,
			     struct bt_gatt_discover_params *params)
{
	int err;

	if (!attr) {
		LOG_WRN("Discovery finished without finding everything");
		memset(params, 0, sizeof(*params));
		return BT_GATT_ITER_STOP;
	}

	if (params->type == BT_GATT_DISCOVER_PRIMARY) {
		struct bt_gatt_service_val *svc = attr->user_data;

		discover_params.uuid = &hake_chr_uuid.uuid;
		discover_params.start_handle = attr->handle + 1;
		discover_params.end_handle = svc->end_handle;
		discover_params.type = BT_GATT_DISCOVER_CHARACTERISTIC;

		err = bt_gatt_discover(conn, &discover_params);
		if (err) {
			LOG_ERR("Char discovery failed (err %d)", err);
		}
	} else if (params->type == BT_GATT_DISCOVER_CHARACTERISTIC) {
		struct bt_gatt_chrc *chrc = attr->user_data;

		subscribe_params.value_handle = chrc->value_handle;
		chr_value_handle = chrc->value_handle;

		discover_params.uuid = BT_UUID_GATT_CCC;
		discover_params.start_handle = chrc->value_handle + 1;
		discover_params.type = BT_GATT_DISCOVER_DESCRIPTOR;

		err = bt_gatt_discover(conn, &discover_params);
		if (err) {
			LOG_ERR("CCC discovery failed (err %d)", err);
		}
	} else {
		subscribe_params.notify = notify_func;
		subscribe_params.value = BT_GATT_CCC_NOTIFY;
		subscribe_params.ccc_handle = attr->handle;

		err = bt_gatt_subscribe(conn, &subscribe_params);
		if (err && err != -EALREADY) {
			LOG_ERR("Subscribe failed (err %d)", err);
		} else {
			atomic_set(&chr_ready, 1);
			LOG_INF("Subscribed, streaming PPG data");
		}
	}

	return BT_GATT_ITER_STOP;
}

static uint8_t read_func(struct bt_conn *conn, uint8_t err,
			 struct bt_gatt_read_params *params,
			 const void *data, uint16_t length)
{
	ARG_UNUSED(conn);
	ARG_UNUSED(params);

	if (err) {
		/* e.g. characteristic not readable: sync still flushes queue */
		LOG_WRN("sync read failed (ATT err 0x%02x)", err);
	} else if (data) {
		decode_ble_payload(data, length);
	}

	atomic_clear(&read_busy);
	return BT_GATT_ITER_STOP;
}

static bool ad_has_name(struct bt_data *data, void *user_data)
{
	bool *found = user_data;

	if (data->type == BT_DATA_NAME_COMPLETE ||
	    data->type == BT_DATA_NAME_SHORTENED) {
		if (data->data_len == strlen(BLE_PERIPHERAL_NAME) &&
		    memcmp(data->data, BLE_PERIPHERAL_NAME, data->data_len) == 0) {
			*found = true;
			return false;
		}
	}
	return true;
}

static void device_found(const bt_addr_le_t *addr, int8_t rssi, uint8_t type,
			 struct net_buf_simple *ad)
{
	struct bt_conn *conn = NULL;
	bool matched = false;
	int err;

	if (default_conn) {
		return;
	}

	if (type != BT_GAP_ADV_TYPE_ADV_IND &&
	    type != BT_GAP_ADV_TYPE_ADV_DIRECT_IND) {
		return;
	}

	bt_data_parse(ad, ad_has_name, &matched);
	if (!matched) {
		return;
	}

	char addr_str[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(addr, addr_str, sizeof(addr_str));
	LOG_INF("Device found: %s (RSSI %d)", addr_str, rssi);

	if (rssi < BLE_MIN_RSSI) {
		return;
	}

	if (bt_le_scan_stop()) {
		return;
	}

	err = bt_conn_le_create(addr, BT_CONN_LE_CREATE_CONN,
				BT_LE_CONN_PARAM_DEFAULT, &conn);
	if (err) {
		LOG_ERR("Create conn to %s failed (%d)", addr_str, err);
		start_scan();
		return;
	}

	k_spinlock_key_t key = k_spin_lock(&conn_lock);

	default_conn = conn;
	k_spin_unlock(&conn_lock, key);
}

static void start_scan(void)
{
	int err = bt_le_scan_start(BT_LE_SCAN_PASSIVE, device_found);

	if (err) {
		LOG_ERR("Scanning failed to start (err %d)", err);
		return;
	}
	LOG_INF("Scanning started");
}

static void mtu_cb(struct bt_conn *conn, uint8_t err,
		   struct bt_gatt_exchange_params *params)
{
	ARG_UNUSED(params);
	LOG_INF("MTU exchange %s, ATT MTU %u", err ? "failed" : "done",
		bt_gatt_get_mtu(conn));
}

static void connected(struct bt_conn *conn, uint8_t err)
{
	if (err) {
		LOG_ERR("Failed to connect: %u %s", err, bt_hci_err_to_str(err));
		clear_default_conn();
		start_scan();
		return;
	}

	if (conn != default_conn) {
		return;
	}

	LOG_INF("Connected: %s", bt_addr_le_str(bt_conn_get_dst(conn)));

	ts_anchored = false;   /* re-anchor sensor clock for this link */

	/* Bigger MTU lets the peripheral batch several samples per packet. */
	mtu_params.func = mtu_cb;
	(void)bt_gatt_exchange_mtu(conn, &mtu_params);

	discover_params.uuid = &hake_svc_uuid.uuid;
	discover_params.func = discover_func;
	discover_params.start_handle = BT_ATT_FIRST_ATTRIBUTE_HANDLE;
	discover_params.end_handle = BT_ATT_LAST_ATTRIBUTE_HANDLE;
	discover_params.type = BT_GATT_DISCOVER_PRIMARY;

	err = bt_gatt_discover(conn, &discover_params);
	if (err) {
		LOG_ERR("Discover failed (err %d)", err);
	}
}

static void disconnected(struct bt_conn *conn, uint8_t reason)
{
	if (conn != default_conn) {
		return;
	}

	LOG_INF("Disconnected, reason 0x%02x %s", reason,
		bt_hci_err_to_str(reason));

	atomic_clear(&chr_ready);
	atomic_clear(&read_busy);
	clear_default_conn();
	start_scan();
}

BT_CONN_CB_DEFINE(conn_callbacks) = {
	.connected = connected,
	.disconnected = disconnected,
};

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
	struct bt_conn *conn = NULL;
	k_spinlock_key_t key = k_spin_lock(&conn_lock);

	if (default_conn && atomic_get(&chr_ready)) {
		conn = bt_conn_ref(default_conn);
	}
	k_spin_unlock(&conn_lock, key);

	/* Buffered samples are flushed by the writer as soon as DTR is up;
	 * additionally request a fresh value from the sensor. */
	if (!conn) {
		LOG_WRN("sync: no sensor connected");
		return;
	}

	if (!atomic_cas(&read_busy, 0, 1)) {
		bt_conn_unref(conn);
		return;   /* a sync read is already in flight */
	}

	read_params.func = read_func;
	read_params.handle_count = 1;
	read_params.single.handle = chr_value_handle;
	read_params.single.offset = 0U;

	int err = bt_gatt_read(conn, &read_params);

	if (err) {
		LOG_WRN("sync: bt_gatt_read failed (%d)", err);
		atomic_clear(&read_busy);
	}
	bt_conn_unref(conn);
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
		LOG_INF("sync requested");
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

	start_scan();

	/* Periodic health log on the debug console (never on the CDC port). */
	for (;;) {
		k_sleep(K_SECONDS(10));
		long d = atomic_clear(&dropped_samples);

		if (d) {
			LOG_WRN("%ld samples dropped (queue full)", d);
		}
	}
	return 0;
}