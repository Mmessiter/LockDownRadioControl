#!/bin/zsh

SRC_RX="/Users/malcolmmessiter/Documents/GitHub/LockDownRadioControl/ReceiverCode/.pio/build/teensy40/firmware.hex"
SRC_TX="/Users/malcolmmessiter/Documents/GitHub/LockDownRadioControl/TransmitterCode/.pio/build/teensy41/firmware.hex"

DST_DIR="/Users/malcolmmessiter/Documents/GitHub/LockDownRadioControl/HEX_Files"
DST_RX="$DST_DIR/RX.HEX"
DST_TX="$DST_DIR/TX.HEX"

if [ ! -f "$SRC_RX" ]; then
    osascript -e 'display dialog "Receiver firmware.hex not found." buttons {"OK"} default button "OK"'
    exit 1
fi

if [ ! -f "$SRC_TX" ]; then
    osascript -e 'display dialog "Transmitter firmware.hex not found." buttons {"OK"} default button "OK"'
    exit 1
fi

mkdir -p "$DST_DIR"

cp "$SRC_RX" "$DST_RX"
cp "$SRC_TX" "$DST_TX"

osascript -e 'display dialog "Done. TX.HEX and RC.HEX have been updated." buttons {"OK"} default button "OK"'