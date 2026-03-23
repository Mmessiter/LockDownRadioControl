#!/bin/zsh

BASE="/Users/malcolmmessiter/Documents/GitHub/LockDownRadioControl"
PROJECT="$BASE/TransmitterCode"
HEX_SOURCE="$BASE/HEX_Files/TX.HEX"
BUILD_HEX="$PROJECT/.pio/build/teensy41/firmware.hex"
PIO="$HOME/.platformio/penv/bin/platformio"
LOG="$BASE/HEX_Files/tx_upload_log.txt"

echo "========================================"
echo "UPLOAD TX.HEX TO TRANSMITTER TEENSY 4.1"
echo "USING PLATFORMIO UPLOAD"
echo "========================================"
echo

if [ ! -f "$HEX_SOURCE" ]; then
    echo "TX.HEX not found:"
    echo "$HEX_SOURCE"
    osascript -e 'display dialog "TX.HEX not found." buttons {"OK"} default button "OK"'
    exit 1
fi

if [ ! -f "$PIO" ]; then
    echo "PlatformIO not found:"
    echo "$PIO"
    osascript -e 'display dialog "PlatformIO not found." buttons {"OK"} default button "OK"'
    exit 1
fi

mkdir -p "$(dirname "$BUILD_HEX")"

echo "Copying TX.HEX into PlatformIO build location..."
cp "$HEX_SOURCE" "$BUILD_HEX"
if [ $? -ne 0 ]; then
    echo "Failed to copy TX.HEX into build folder."
    osascript -e 'display dialog "Failed to copy TX.HEX into build folder." buttons {"OK"} default button "OK"'
    exit 1
fi

echo
echo "Using source HEX:"
echo "$HEX_SOURCE"
echo
echo "Upload HEX path:"
echo "$BUILD_HEX"
echo
echo "Log file:"
echo "$LOG"
echo

cd "$PROJECT" || exit 1

echo "Starting PlatformIO upload..."
echo

"$PIO" run -e teensy41 -t nobuild -t upload | tee "$LOG"
RESULT=${pipestatus[1]}

echo
echo "PlatformIO exit code = $RESULT"
echo

if [ $RESULT -eq 0 ]; then
    echo "Done. TX.HEX has been loaded into the Teensy 4.1."
    osascript -e 'display dialog "Done. TX.HEX has been loaded into the Teensy 4.1." buttons {"OK"} default button "OK"'
else
    echo "Upload failed."
    echo "If the current firmware is not responding over USB, the PROGRAM button may still be needed."
    osascript -e 'display dialog "Upload failed. If the current firmware is not responding over USB, the PROGRAM button may still be needed." buttons {"OK"} default button "OK"'
fi

exit 0