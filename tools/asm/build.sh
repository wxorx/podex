#!/bin/sh
# Assemble the HC12 routines; requires m68hc11-as (m68hc1x toolchain).
set -e
cd "$(dirname "$0")"
for f in calib flash_erase flash_prog ee_erase ee_prog; do
    m68hc11-as -m68hc12 -o "$f.o" "$f.s"
    m68hc11-objcopy -O binary "$f.o" "$f.bin"
    size=$(stat -c %s "$f.bin")
    echo "$f.bin: $size bytes"
    if [ "$size" -gt 384 ]; then
        echo "ERROR: $f.bin too large for $2000-$217F code area" >&2
        exit 1
    fi
done
