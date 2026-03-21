#!/bin/zsh

BASE="/Users/malcolmmessiter/Documents/GitHub/LockDownRadioControl"
TX_DIR="$BASE/TransmitterCode"
RX_DIR="$BASE/ReceiverCode"
DST_DIR="$BASE/HEX_Files"

DST_TX="$DST_DIR/TX.HEX"
DST_RX="$DST_DIR/RX.HEX"

PIO="$HOME/.platformio/penv/bin/platformio"

echo "========================================"
echo "CLEAN REBUILD: TRANSMITTER"
echo "========================================"
cd "$TX_DIR" || exit 1

"$PIO" run --target clean
if [ $? -ne 0 ]; then
    echo
    echo "Transmitter clean FAILED."
    osascript -e 'display dialog "Transmitter clean FAILED." buttons {"OK"} default button "OK"'
    exit 1
fi

"$PIO" run
if [ $? -ne 0 ]; then
    echo
    echo "Transmitter build FAILED."
    osascript -e 'display dialog "Transmitter build FAILED." buttons {"OK"} default button "OK"'
    exit 1
fi

echo
echo "========================================"
echo "CLEAN REBUILD: RECEIVER"
echo "========================================"
cd "$RX_DIR" || exit 1

"$PIO" run --target clean
if [ $? -ne 0 ]; then
    echo
    echo "Receiver clean FAILED."
    osascript -e 'display dialog "Receiver clean FAILED." buttons {"OK"} default button "OK"'
    exit 1
fi

"$PIO" run
if [ $? -ne 0 ]; then
    echo
    echo "Receiver build FAILED."
    osascript -e 'display dialog "Receiver build FAILED." buttons {"OK"} default button "OK"'
    exit 1
fi

echo
echo "========================================"
echo "FINDING NEW firmware.hex FILES"
echo "========================================"

SRC_TX=$(find "$TX_DIR/.pio/build" -name firmware.hex | head -n 1)
SRC_RX=$(find "$RX_DIR/.pio/build" -name firmware.hex | head -n 1)

echo "Transmitter hex:"
echo "$SRC_TX"
echo
echo "Receiver hex:"
echo "$SRC_RX"
echo

if [ -z "$SRC_TX" ] || [ ! -f "$SRC_TX" ]; then
    echo "Transmitter firmware.hex not found."
    osascript -e 'display dialog "Transmitter firmware.hex not found." buttons {"OK"} default button "OK"'
    exit 1
fi

if [ -z "$SRC_RX" ] || [ ! -f "$SRC_RX" ]; then
    echo "Receiver firmware.hex not found."
    osascript -e 'display dialog "Receiver firmware.hex not found." buttons {"OK"} default button "OK"'
    exit 1
fi

mkdir -p "$DST_DIR"

echo "Copying transmitter firmware to:"
echo "$DST_TX"
cp "$SRC_TX" "$DST_TX"
if [ $? -ne 0 ]; then
    echo "Failed to copy TX.HEX"
    osascript -e 'display dialog "Failed to copy TX.HEX" buttons {"OK"} default button "OK"'
    exit 1
fi

echo
echo "Copying receiver firmware to:"
echo "$DST_RX"
cp "$SRC_RX" "$DST_RX"
if [ $? -ne 0 ]; then
    echo "Failed to copy RX.HEX"
    osascript -e 'display dialog "Failed to copy RX.HEX" buttons {"OK"} default button "OK"'
    exit 1
fi

echo
echo "Done. Full rebuild completed."
echo "TX.HEX and RX.HEX have been updated."

osascript -e 'display dialog "Done. Full rebuild completed. TX.HEX and RX.HEX have been updated." buttons {"OK"} default button "OK"'