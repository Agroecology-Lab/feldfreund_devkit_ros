#!/bin/bash
# 🦎 Official Lizard Flasher (Agroecology Lab Specialist Version)

# 1. Hardware Access & Environment Validation
if [ -f "fixusb.py" ]; then
    python3 fixusb.py
else
    echo "⚠️ Warning: fixusb.py not found."
fi

if [ -f ".env" ]; then
    export $(grep -v '^#' .env | xargs)
else
    echo "❌ Error: .env file missing."
    exit 1
fi

# Prioritise .env MCU_PORT
MCU_PORT=${MCU_PORT:-$(ls /dev/ttyACM* 2>/dev/null | head -n 1)}

# 2. Binary Path Resolution (Validated from Environment Scan)
BOOTLOADER="./src/basekit_driver/build/bootloader/bootloader.bin"
PARTITIONS="./src/basekit_driver/build/partition_table/partition-table.bin"
FIRMWARE="./firmware.bin"

# Check for existence before proceeding
for bin in "$BOOTLOADER" "$PARTITIONS" "$FIRMWARE"; do
    if [ ! -f "$bin" ]; then
        echo "❌ Error: Required binary $bin missing."
        exit 1
    fi
done

# 3. Virtual Environment Entry
if [ -d ".lizard_venv" ]; then
    source .lizard_venv/bin/activate
fi

echo "🚀 Starting Lizard Flash on $MCU_PORT..."

# 4. Official Lizard/Zauberzeug Flash Sequence (Modern esptool v5.1+)
# S3 Spec: Bootloader @ 0x0 | Partitions @ 0x8000 | Firmware @ 0x20000
python3 -m esptool --chip esp32s3 --port "$MCU_PORT" --baud 921600 \
    --before default-reset --after hard-reset write-flash \
    -z --flash-mode dio --flash-freq 80m --flash-size detect \
    0x0 "$BOOTLOADER" \
    0x8000 "$PARTITIONS" \
    0x20000 "$FIRMWARE"

# 5. The "Lizard Kick" (Critical for ThinkPad X1 USB stability)
echo "⚡ Stabilising TTY and triggering hardware boot..."
sudo stty -F "$MCU_PORT" -hupcl raw 115200
python3 -c "import serial, time; s=serial.Serial('$MCU_PORT'); s.dtr=0; s.rts=0; time.sleep(0.5); s.dtr=1; s.rts=1; s.close()"

# 6. Final Verification Audit
if [ -f "./espinfo.sh" ]; then
    echo "🔍 Running espinfo.sh for final audit..."
    ./espinfo.sh
fi
