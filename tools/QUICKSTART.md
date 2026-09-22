# Quick start — erase & flash the MC912DG128A via podex

Hardware: podex on `/dev/ttyUSB0`, target powered. Nothing else needed.

```sh
cd ./tools

# Optional one-time speedup (persists until replug/reboot):
echo 1 | sudo tee /sys/bus/usb-serial/devices/ttyUSB0/latency_timer
```

## The one command you usually want

```sh
python3 podex_programmer.py restore --yes
```

Erases + rewrites + verifies **both** flash (128K) and EEPROM (2K) from
`ref_bins/`, then checks md5. ~3 min. Ends with:

```
[+] eeprom md5 4702072b19b5a9c0b0ba3bc192476e05 == REF (PASS)
[+] flash  md5 c23d85d31bafe6ca75e97fe196b2b9bd == REF (PASS)
```

No `--yes` = it asks before erasing.

## Single tasks

```sh
# erase all flash (add --modules 0xN for a single module bitmask)
python3 podex_programmer.py erase-flash --yes

# flash a raw 128K image (prompts to erase first; skips erased $FF rows)
python3 podex_programmer.py write-flash my_image.bin

# read back / verify (compare to reference dump)
python3 podex_programmer.py read-flash out.bin --ref
python3 podex_programmer.py verify-flash my_image.bin

# EEPROM variants
python3 podex_programmer.py erase-eeprom --yes          # bulk
python3 podex_programmer.py write-eeprom my_eeprom.bin
python3 podex_programmer.py read-eeprom out.bin --ref
python3 podex_programmer.py verify-eeprom my_eeprom.bin

# what is connected / registers / measured clock
python3 podex_programmer.py probe
```

Both `write-*` commands verify after programming and fail loudly on any
mismatch. Images must be raw binaries (128K flash / 2K EEPROM), same
format as `ref_bins/*.bin`.

## Troubleshooting

- `podex is dead` / CTS timeouts → wrong port, or re-plug USB; retry.
- Weird failures after an aborted run → `python3 podex_programmer.py reset`,
  then retry.
- Full flash rewrite takes ~2 min with the latency tweak, ~10 min without.

Background, silicon quirks and internals: `DG128A_flash_notes.md`
(same directory). Source of the flash routines: `asm/*.s`.
