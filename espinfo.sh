#!/bin/bash

# 1. Hardware Access & Environment Setup
if [ -f "fixusb.py" ]; then
    python3 fixusb.py
fi

# Determine Python path from .lizard_venv or default venv
if [ -d ".lizard_venv" ]; then
    VENV_PYTHON="$(pwd)/.lizard_venv/bin/python3"
elif [ -d "venv" ]; then
    VENV_PYTHON="$(pwd)/venv/bin/python3"
else
    echo "Error: Virtual environment not found (.lizard_venv or venv)."
    exit 1
fi

# Load environment variables
if [ -f ".env" ]; then
    export $(grep -v '^#' .env | xargs)
fi

# Priority: .env MCU_PORT > Auto-detect
MCU_PORT=${MCU_PORT:-$(ls /dev/ttyACM* 2>/dev/null | head -n 1)}

if [ -z "$MCU_PORT" ]; then
    echo "Error: No ESP32-S3 detected on /dev/ttyACM*."
    exit 1
fi

echo "--- ESP32-S3 Interrogation: $MCU_PORT ---"

# 2. Extract Remote State
echo "[📡] Reading flash state..."
$VENV_PYTHON -m esptool --chip esp32s3 --port "$MCU_PORT" read_flash 0x8000 0xC00 dumped_partitions.bin > /dev/null 2>&1
$VENV_PYTHON -m esptool --chip esp32s3 --port "$MCU_PORT" read_flash 0xd000 0x20 ota_state.bin > /dev/null 2>&1
$VENV_PYTHON -m esptool --chip esp32s3 --port "$MCU_PORT" read_flash 0x20000 16 app_header.bin > /dev/null 2>&1

# 3. Comprehensive Analysis via Python
$VENV_PYTHON -c "
import struct
import os

def parse():
    print(f'\n[Partition Table]')
    print(f'{\"LABEL\":<12} | {\"OFFSET\":<10} | {\"SIZE\":<10}')
    print('-' * 40)
    if os.path.exists('dumped_partitions.bin'):
        with open('dumped_partitions.bin', 'rb') as f:
            data = f.read()
            for i in range(0, len(data), 32):
                chunk = data[i:i+32]
                if len(chunk) >= 32 and chunk[0:2] == b'\xaa\x50':
                    label = chunk[12:28].strip(b'\x00').decode('utf-8', 'ignore')
                    off = struct.unpack('<I', chunk[4:8])[0]
                    size = struct.unpack('<I', chunk[8:12])[0]
                    print(f'{label:<12} | 0x{off:08x} | 0x{size:08x}')

    if os.path.exists('ota_state.bin'):
        with open('ota_state.bin', 'rb') as f:
            raw = f.read(4)
            if len(raw) == 4:
                seq = struct.unpack('<I', raw)[0]
                active = 'ota_0' if seq == 1 else 'ota_1' if seq == 2 else 'factory'
                print(f'\n[OTA Status]\n  Sequence: {seq} (Active: {active})')

    print(f'\n[Firmware Verification]')
    if os.path.exists('app_header.bin'):
        with open('app_header.bin', 'rb') as f:
            header = f.read(16)
            magic = header[0:1].hex()
            print(f'  Remote Magic Byte: {magic} (Valid: {\"YES\" if magic==\"e9\" else \"NO\"})')
            if os.path.exists('firmware.bin'):
                with open('firmware.bin', 'rb') as f2:
                    if header == f2.read(16):
                        print('  Match: Local firmware.bin matches Remote (0x20000).')
                    else:
                        print('  Mismatch: Local firmware.bin differs from Flashed firmware.')
parse()
"

# 4. TTY Stabilisation & Hardware Reset
echo -e "\n--- Hardware Reset & TTY Stabilisation ---"
sudo stty -F "$MCU_PORT" -hupcl raw 115200
$VENV_PYTHON -c "import serial, time; s=serial.Serial('$MCU_PORT'); s.dtr=False; s.rts=False; time.sleep(0.5); s.dtr=True; s.rts=True; time.sleep(0.1); s.close()"

# 5. Log Capture with Pure-Bash Timestamping
LOG_FILE="boot_debug_$(date +%s).log"
echo "Logging started to $LOG_FILE. Press Ctrl+C to stop."

# Capture and timestamp
cat "$MCU_PORT" | while IFS= read -r line; do
    printf "[%(%Y-%m-%d %H:%M:%S)T] %s\n" -1 "$line"
done | tee "$LOG_FILE"

# Cleanup
rm -f dumped_partitions.bin ota_state.bin app_header.bin
