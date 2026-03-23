#!/bin/zsh

BASE="/Users/malcolmmessiter/Documents/GitHub/LockDownRadioControl"
HEX="$BASE/HEX_Files/TX.HEX"

TOOLS="/Users/malcolmmessiter/.platformio/packages/tool-teensy"
LOADER="$TOOLS/teensy_loader_cli"
REBOOT="$TOOLS/teensy_reboot"
LOG="$BASE/HEX_Files/teensy_upload_log.txt"

echo "========================================"
echo "PROGRAMMING TRANSMITTER TEENSY 4.1"
echo "========================================"

if [ ! -f "$HEX" ]; then
    echo "TX.HEX not found:"
    echo "$HEX"
    osascript -e 'display dialog "TX.HEX not found." buttons {"OK"} default button "OK"'
    exit 1
fi

if [ ! -f "$LOADER" ]; then
    echo "teensy_loader_cli not found:"
    echo "$LOADER"
    osascript -e 'display dialog "teensy_loader_cli not found." buttons {"OK"} default button "OK"'
    exit 1
fi

echo "Using HEX file:"
echo "$HEX"
echo
echo "Using loader:"
echo "$LOADER"
echo
echo "Log file:"
echo "$LOG"
echo

if [ -f "$REBOOT" ]; then
    echo "Trying to reboot Teensy into bootloader..."
    "$REBOOT" >/dev/null 2>&1
    sleep 1
fi

echo "Waiting for Teensy 4.1..."
echo "If needed, press the PROGRAM button on the Teensy."
echo

"$LOADER" --mcu=imxrt1062 -w -v "$HEX" | tee "$LOG"
RESULT=$?

echo
echo "teensy_loader_cli exit code = $RESULT"
echo

if [ $RESULT -eq 0 ]; then
    echo "Done. TX.HEX has been loaded into the Teensy 4.1."
    osascript -e 'display dialog "Done. TX.HEX has been loaded into the Teensy 4.1." buttons {"OK"} default button "OK"'
else
    echo "The loader reported a problem, but the Teensy may still have been programmed successfully."
    echo "Please check whether the new firmware is actually running."
    osascript -e 'display dialog "Loader reported a problem. The Teensy may still have been programmed successfully." buttons {"OK"} default button "OK"'
fi

exit 0