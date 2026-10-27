/*
 * USB device bring-up for the new Zephyr USB device stack (USBD / device_next).
 * Exposes one native CDC ACM function: a communication (control) interface
 * with an interrupt notification endpoint, plus a data interface with
 * bulk IN / bulk OUT endpoints. Grouped with an IAD so composite hosts
 * (incl. Android) bind it as a standard CDC ACM device.
 */
#include <zephyr/kernel.h>
#include <zephyr/usb/usbd.h>
#include <zephyr/usb/usb_ch9.h>
#include <zephyr/logging/log.h>

#include "usb_dev.h"

LOG_MODULE_REGISTER(ppg_usb, LOG_LEVEL_INF);

/* 0x2fe3 is the Zephyr project test VID. Replace with your own VID/PID
 * before shipping. */
#define PPG_USB_VID 0x2fe3
#define PPG_USB_PID 0x0001

USBD_DEVICE_DEFINE(ppg_usbd, DEVICE_DT_GET(DT_NODELABEL(zephyr_udc0)),
		   PPG_USB_VID, PPG_USB_PID);

USBD_DESC_LANG_DEFINE(ppg_lang);
USBD_DESC_MANUFACTURER_DEFINE(ppg_mfr, "Example Corp");
USBD_DESC_PRODUCT_DEFINE(ppg_product, "PPG Sensor (CDC)");
USBD_DESC_SERIAL_NUMBER_DEFINE(ppg_sn);

USBD_DESC_CONFIG_DEFINE(fs_cfg_desc, "FS Configuration");
USBD_DESC_CONFIG_DEFINE(hs_cfg_desc, "HS Configuration");

/* Bus powered, 100 mA. Use USB_SCD_SELF_POWERED if appropriate. */
USBD_CONFIGURATION_DEFINE(ppg_fs_config, 0, 100, &fs_cfg_desc);
USBD_CONFIGURATION_DEFINE(ppg_hs_config, 0, 100, &hs_cfg_desc);

static usbd_msg_cb_t user_cb;

static void ppg_usbd_msg_cb(struct usbd_context *const ctx,
			    const struct usbd_msg *msg)
{
	if (usbd_can_detect_vbus(ctx)) {
		if (msg->type == USBD_MSG_VBUS_READY) {
			if (usbd_enable(ctx)) {
				LOG_ERR("usbd_enable failed");
			}
		} else if (msg->type == USBD_MSG_VBUS_REMOVED) {
			if (usbd_disable(ctx)) {
				LOG_ERR("usbd_disable failed");
			}
		}
	}

	if (user_cb) {
		user_cb(ctx, msg);
	}
}

static int add_speed(enum usbd_speed speed, struct usbd_config_node *cfg)
{
	int err;

	err = usbd_add_configuration(&ppg_usbd, speed, cfg);
	if (err) {
		LOG_ERR("add configuration (%d)", err);
		return err;
	}

	/* Registers every class instance enabled in DT; we only have one
	 * cdc-acm-uart node, so this yields exactly one CDC function. */
	err = usbd_register_all_classes(&ppg_usbd, speed, 1, NULL);
	if (err) {
		LOG_ERR("register classes (%d)", err);
		return err;
	}

	/* Miscellaneous / Common / IAD so the CDC interface pair is grouped. */
	usbd_device_set_code_triple(&ppg_usbd, speed,
				    USB_BCC_MISCELLANEOUS, 0x02, 0x01);
	return 0;
}

int ppg_usb_init(usbd_msg_cb_t cb)
{
	int err;

	user_cb = cb;

	err = usbd_add_descriptor(&ppg_usbd, &ppg_lang) ||
	      usbd_add_descriptor(&ppg_usbd, &ppg_mfr) ||
	      usbd_add_descriptor(&ppg_usbd, &ppg_product) ||
	      usbd_add_descriptor(&ppg_usbd, &ppg_sn);
	if (err) {
		LOG_ERR("add string descriptors failed");
		return -EIO;
	}

	if (usbd_caps_speed(&ppg_usbd) == USBD_SPEED_HS) {
		err = add_speed(USBD_SPEED_HS, &ppg_hs_config);
		if (err) {
			return err;
		}
	}

	err = add_speed(USBD_SPEED_FS, &ppg_fs_config);
	if (err) {
		return err;
	}

	err = usbd_msg_register_cb(&ppg_usbd, ppg_usbd_msg_cb);
	if (err) {
		return err;
	}

	err = usbd_init(&ppg_usbd);
	if (err) {
		LOG_ERR("usbd_init (%d)", err);
		return err;
	}

	/* Without VBUS detection, enable immediately; otherwise wait for
	 * USBD_MSG_VBUS_READY in the callback above. */
	if (!usbd_can_detect_vbus(&ppg_usbd)) {
		err = usbd_enable(&ppg_usbd);
		if (err) {
			LOG_ERR("usbd_enable (%d)", err);
			return err;
		}
	}

	LOG_INF("USB CDC ACM device initialized");
	return 0;
}