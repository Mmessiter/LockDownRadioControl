#!/bin/zsh

BASE="/Users/malcolmmessiter/Documents/GitHub/LockDownRadioControl"
PROJECT="$BASE/ReceiverCode"
HEX_SOURCE="$BASE/HEX_Files/RX.HEX"
BUILD_HEX="$PROJECT/.pio/build/teensy40/firmware.hex"

PIO="$HOME/.platformio/penv/bin/platformio"

TOOLS="/Users/malcolmmessiter/.platformio/packages/tool-teensy"
LOADER="$TOOLS/teensy_loader_cli"
REBOOT="$TOOLS/teensy_reboot"

LOG="$BASE/HEX_Files/rx_upload_log.txt"

echo "========================================"
echo "UPLOAD RX.HEX TO RECEIVER TEENSY 4.0"
echo "========================================"
echo

if [ ! -f "$HEX_SOURCE" ]; then
    echo "RX.HEX not found:"
    echo "$HEX_SOURCE"
    osascript -e 'display dialog "RX.HEX not found." buttons {"OK"} default button "OK"'
    exit 1
fi

echo "Using source HEX:"
echo "$HEX_SOURCE"
echo
echo "Log file:"
echo "$LOG"
echo

mkdir -p "$(dirname "$BUILD_HEX")"

echo "Copying RX.HEX into PlatformIO build location..."
cp "$HEX_SOURCE" "$BUILD_HEX"
if [ $? -ne 0 ]; then
    echo "Failed to copy RX.HEX into build folder."
    osascript -e 'display dialog "Failed to copy RX.HEX into build folder." buttons {"OK"} default button "OK"'
    exit 1
fi

echo
echo "PlatformIO upload path:"
echo "$BUILD_HEX"
echo

if [ -f "$PIO" ]; then
    echo "========================================"
    echo "ATTEMPT 1: PLATFORMIO UPLOAD"
    echo "========================================"
    echo

    cd "$PROJECT" || exit 1
    "$PIO" run -e teensy40 -t nobuild -t upload | tee "$LOG"
    PIO_RESULT=${pipestatus[1]}

    echo
    echo "PlatformIO exit code = $PIO_RESULT"
    echo

    if [ $PIO_RESULT -eq 0 ]; then
        echo "Done. RX.HEX has been loaded into the Teensy 4.0 by PlatformIO."
        osascript -e 'display dialog "Done. RX.HEX has been loaded into the Teensy 4.0." buttons {"OK"} default button "OK"'
        exit 0
    fi
fi

echo "========================================"
echo "ATTEMPT 2: ASK RUNNING CODE TO REBOOT"
echo "========================================"
echo

if [ -f "$REBOOT" ]; then
    "$REBOOT" >/dev/null 2>&1
    sleep 2
fi

echo "========================================"
echo "ATTEMPT 3: DIRECT LOADER, SHORT WAIT"
echo "========================================"
echo

if [ ! -f "$LOADER" ]; then
    echo "teensy_loader_cli not found:"
    echo "$LOADER"
    osascript -e 'display dialog "teensy_loader_cli not found." buttons {"OK"} default button "OK"'
    exit 1
fi

echo "If the receiver does not enter bootloader by itself,"
echo "you will need to press the PROGRAM button."
echo

"$LOADER" --mcu=imxrt1062 -w -s -v "$HEX_SOURCE" | tee -a "$LOG" &
LOADER_PID=$!

sleep 6

if kill -0 $LOADER_PID 2>/dev/null; then
    kill $LOADER_PID 2>/dev/null
    wait $LOADER_PID 2>/dev/null

    echo
    echo "Receiver did not auto-enter programming mode."
    echo "Please press the PROGRAM button on the Teensy, then run r.command again."
    osascript -e 'display dialog "Receiver did not auto-enter programming mode. Please press the PROGRAM button on the Teensy, then run r.command again." buttons {"OK"} default button "OK"'
    exit 1
fi

wait $LOADER_PID
LOADER_RESULT=$?

echo
echo "teensy_loader_cli exit code = $LOADER_RESULT"
echo

if [ $LOADER_RESULT -eq 0 ]; then
    echo "Done. RX.HEX has been loaded into the Teensy 4.0."
    osascript -e 'display dialog "Done. RX.HEX has been loaded into the Teensy 4.0." buttons {"OK"} default button "OK"'
else
    echo "Upload failed."
    echo "This receiver probably cannot auto-reboot into programming mode."
    osascript -e 'display dialog "Upload failed. This receiver probably cannot auto-reboot into programming mode, so the PROGRAM button is needed." buttons {"OK"} default button "OK"'
fi

exit 0