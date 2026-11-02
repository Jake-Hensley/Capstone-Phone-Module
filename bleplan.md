# Sending test BLE advertisements from the laptop

The laptop (Intel AX211 Bluetooth adapter, BlueZ 5.77) can pretend to be the sensor. `scripts/fake_sensor.py` sends advertisements in the exact format the bridge firmware expects (see `plan.md`, section 1).

## What the script does

- Every 2 seconds it builds a new advertisement with the name `BEACON`, company ID `0xFFFF`, version `01` and one sample.
- The timestamp is milliseconds since the script started.
- Red and IR are a made-up pulse at about 72 bpm.
- It sends the advertisement as non-connectable, so it shows up as the broadcast type the firmware accepts.
- Ctrl+C stops it and removes the advertisement.

## Packet layout (28 of 31 bytes)

```
02 01 06                         Flags
07 09 42 45 41 43 4F 4E          Complete name "BEACON"
10 FF  FF FF  01  <12 bytes>     Manufacturer data: company 0xFFFF, version 0x01,
                                 then timestamp (ms), red, IR as uint32 little-endian
```

Example from a dry run:
```
t=1000 ms  red=103806  ir=135709  adv=0201060709424541434F4E10FFFFFF01E80300007E9501001D120200
```

## Running it

Use a separate terminal, since it keeps running until you stop it. Sending needs root:
```
sudo python3 ~/Capstone/scripts/fake_sensor.py
```

To print the packets without sending anything:
```
python3 ~/Capstone/scripts/fake_sensor.py --dry-run
```

Options:
- `--period <seconds>`: time between new samples (default 2.0)
- `--instance <id>`: advertising slot to use (default 5)

## Checking it's working

1. On the board's debug log, look for `Sensor found: ...` once, then every 10 s `last 10 s: N matching advertisements, ~5 new samples`.
2. Open the USB port with `tio /dev/ttyACM0`. You should see a JSON line about every 2 s.

## Notes

- **Slow on purpose:** Linux advertises about every 1.28 s by default, so the script sends a new sample every 2 s. Change it with `--period`. The sample rate doesn't matter for testing.
- **Signal strength:** keep the laptop close to the board, because the firmware ignores signals weaker than -70 dBm.
- **If nothing shows up:** run `sudo btmon` in another terminal. If the laptop is really sending, it shows the advertising commands going out.
- **Phones can't do this:** nRF Connect on an iPhone can only advertise a name and service UUIDs, not manufacturer data. Android's nRF Connect can.
- **Tested so far:** only `--dry-run`. Real broadcasting hasn't been tried yet.
