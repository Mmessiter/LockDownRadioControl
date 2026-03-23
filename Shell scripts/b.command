#!/bin/zsh

BASE="/Users/malcolmmessiter/Documents/GitHub/LockDownRadioControl"

SRC_RX=$(find "$BASE/ReceiverCode/.pio/build" -name firmware.hex | head -n 1)
SRC_TX=$(find "$BASE/TransmitterCode/.pio/build" -name firmware.hex | head -n 1)

DST_DIR="$BASE/HEX_Files"
DST_RX="$DST_DIR/RX.HEX"
DST_TX="$DST_DIR/TX.HEX"

echo "Looking for transmitter firmware.hex..."
echo "Found: $SRC_TX"
echo

echo "Looking for receiver firmware.hex..."
echo "Found: $SRC_RX"
echo

if [ -z "$SRC_RX" ] || [ ! -f "$SRC_RX" ]; then
    echo "Receiver firmware.hex not found."
    osascript -e 'display dialog "Receiver firmware.hex not found." buttons {"OK"} default button "OK"'
    exit 1
fi

if [ -z "$SRC_TX" ] || [ ! -f "$SRC_TX" ]; then
    echo "Transmitter firmware.hex not found."
    osascript -e 'display dialog "Transmitter firmware.hex not found." buttons {"OK"} default button "OK"'
    exit 1
fi

mkdir -p "$DST_DIR"

echo "Copying transmitter firmware to:"
echo "$DST_TX"
cp "$SRC_TX" "$DST_TX"
echo

echo "Copying receiver firmware to:"
echo "$DST_RX"
cp "$SRC_RX" "$DST_RX"
echo

echo "Done. TX.HEX and RX.HEX have been updated."

osascript -e 'display dialog "Done. TX.HEX and RX.HEX have been updated." buttons {"OK"} default button "OK"'