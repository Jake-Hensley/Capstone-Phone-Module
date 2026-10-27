#ifndef PPG_USB_DEV_H_
#define PPG_USB_DEV_H_

#include <zephyr/usb/usbd.h>

/* Bring up the USBD stack with one CDC ACM function. cb receives every
 * USBD message (line state, line coding, reset, VBUS...). */
int ppg_usb_init(usbd_msg_cb_t cb);

#endif /* PPG_USB_DEV_H_ */