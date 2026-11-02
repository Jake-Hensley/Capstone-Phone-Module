# Plan: read PPG data from advertising packets instead of a connection

The receiving board just listens and never connects. The sensor puts one sample in each advertisement and updates it whenever it has a new one. The USB side stays exactly as it is.

Assumptions: the sample rate doesn't matter, and some lost packets are acceptable.

## 1. Agree on the packet format (with whoever writes the sensor firmware)

Standard advertisements hold 31 bytes. Use the **manufacturer-specific data** field and keep the existing 12-byte sample record, so `decode_ble_payload()` works unchanged:

| Field | Bytes | Notes |
|---|---|---|
| Flags | 3 | standard |
| Name `"Hake"` | 6 | still used to tell our sensor apart |
| Manufacturer data header | 2 | length and type `0xFF` |
| Company ID `0xFFFF` | 2 | the ID reserved for testing, not a real company |
| Format version | 1 | e.g. `0x01`, so the format can change later |
| Record: timestamp, red, IR (4 bytes each, little-endian) | 12 | same layout as now |
| **Total** | **26 of 31** | |

Rules for the sensor:
- Advertise as **non-connectable**, and put the name in the main packet, not the scan response.
- After each new sample, replace the advertisement contents with `bt_le_adv_update_data()`.
- **The timestamp must go up with every new sample.** The receiver hears the same advertisement many times and uses the timestamp to tell new samples from repeats.
- Use a fixed address (no address rotation), so the receiver can lock onto it if wanted.

## 2. `app/prj.conf`

- Remove `CONFIG_BT_CENTRAL`, `CONFIG_BT_GATT_CLIENT` and the three buffer/MTU size lines. They only matter for a connection.
- Add `CONFIG_BT_OBSERVER=y`, which is the listen-only role.
- Leave the USB and app settings alone.

## 3. `app/CMakeLists.txt` (existing bug, needed for it to build at all)

Replace `target_sources(app PRIVATE inc)` with `target_include_directories(app PRIVATE inc)`.

## 4. `app/src/main.c`

**a. Settings (top of the file)**
- Remove `hake_svc_uuid` and `hake_chr_uuid`.
- Add `PPG_COMPANY_ID 0xFFFF` and `PPG_ADV_VERSION 0x01`.
- Loosen `BLE_MIN_RSSI` to around `-90`, or remove the check. At -20 dBm the sensor would almost have to touch the board.

**b. Delete the connection code.** All of the following goes away:
- `default_conn`, `conn_lock`, `chr_ready`, `chr_value_handle`
- the discover, subscribe, MTU and read parameter structs, and `read_busy`
- `clear_default_conn`, `notify_func`, `discover_func`, `read_func`, `mtu_cb`
- `connected`, `disconnected` and `BT_CONN_CB_DEFINE`

**c. Read the advertisement in one pass.** Replace `ad_has_name()` with a function that records both whether the name matched and where the manufacturer data is:
```c
struct adv_info {
	bool name_ok;
	const uint8_t *mfr;   /* only valid while device_found() is running */
	uint8_t mfr_len;
};
```
- For a name field: compare it to `BLE_PERIPHERAL_NAME`, as the code does now.
- For a manufacturer data field: require at least 3 bytes, the company ID `0xFFFF` (read with `sys_get_le16`), and the right version byte. If all three pass, save a pointer to the bytes after the version byte.
- Keep going until every field has been checked.

**d. Rewrite `device_found()`**
- Accept the non-connectable advertisement types (`BT_GAP_ADV_TYPE_ADV_NONCONN_IND` and `BT_GAP_ADV_TYPE_ADV_SCAN_IND`), plus `ADV_IND` in case the sensor ends up connectable.
- Read the packet as in 4c. If the name doesn't match or there's no valid manufacturer data, ignore it.
- Apply the signal-strength check.
- Optionally lock onto the first matching address (save it with `bt_addr_le_copy` and ignore other addresses), so a second "Hake" nearby can't mix in its samples.
- Call `decode_ble_payload(info.mfr, info.mfr_len)`. Decode right away, inside the callback, because the packet buffer isn't kept afterwards.
- Never stop scanning and never connect.

**e. `start_scan()`**
- Scan with duplicate filtering **off**. Otherwise the radio chip may only report the first packet from the sensor and drop the rest.
- Make the scan window equal the interval, so the radio listens all the time:
  ```c
  bt_le_scan_start(BT_LE_SCAN_PARAM(BT_LE_SCAN_TYPE_PASSIVE,
                   BT_LE_SCAN_OPT_NONE, 0x0060, 0x0060), device_found);
  ```
- It's called once from `main()`. Nothing needs to restart it, because there's no disconnect.

**f. Timestamp handling**
- Keep it as is. `device_found()` runs on the same Bluetooth thread the notification callback used, so no locking changes are needed.
- Repeated copies of an advertisement have a timestamp equal to the last one, so `map_timestamp()` already drops them.
- The line in `connected()` that reset the timestamp offset goes away. The existing "clock went backwards by more than 1 s" check still catches a sensor reboot.
- Edge case: if the sensor reboots within 1 s of its last sample, a few samples just after the reboot are dropped as repeats. That's fine given that lost packets are OK.

**g. The `sync` command from the PC**
- It can no longer ask the sensor for a reading. Keep the command so the PC program doesn't break, but have `handle_sync()` only log it.
- The USB writer already sends any held samples as soon as the PC opens the port.

**h. Leave alone:** the sample queue, `decode_ble_payload()`, everything on the USB side, and the line reassembly for PC commands.

**i. Housekeeping**
- Update the comment at the top of the file to describe the new setup.
- Add a "matching advertisements / new samples" count to the 10-second health log, so you can see the actual sample rate during testing.

## 5. Testing

1. **Without the real sensor:** the nRF Connect phone app can send a custom advertisement with name `Hake`, company ID `0xFFFF` and a 13-byte payload (version byte plus one record). Change the timestamp bytes by hand and check that a new JSON line appears.
2. On the PC, run `tio /dev/ttyACM0` and confirm one line per *new* timestamp, with no repeats.
3. Check that the debug log shows advertisements arriving well above the sample rate. That shows duplicate filtering is really off.
4. Reboot the sensor while streaming. Check that `t_ms` keeps going up.
5. Add a second sender with a different name or company ID. Check that it's ignored.

## Later, if you need more speed

Turning on Bluetooth 5 extended advertising (`CONFIG_BT_EXT_ADV` on both sides) allows packets of up to about 250 bytes, so roughly 20 samples per packet. `decode_ble_payload()` already handles several records in a row, so the main change would be in the scan code.
