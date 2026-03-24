#!/bin/zsh
setopt NULL_GLOB

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
LOG="$SCRIPT_DIR/tx_upload_log.txt"

echo "========================================"
echo "UPLOAD HEX TO TRANSMITTER TEENSY 4.1"
echo "========================================"
echo
echo "Running from:"
echo "$SCRIPT_DIR"
echo

find_hex_file() {
    local matches=()

    for f in \
        "$SCRIPT_DIR/TX.HEX" \
        "$SCRIPT_DIR/TX.hex" \
        "$SCRIPT_DIR/Tx.HEX" \
        "$SCRIPT_DIR/Tx.hex" \
        "$SCRIPT_DIR/tx.HEX" \
        "$SCRIPT_DIR/tx.hex"
    do
        if [ -f "$f" ]; then
            echo "$f"
            return 0
        fi
    done

    matches=( "$SCRIPT_DIR"/*.hex "$SCRIPT_DIR"/*.HEX )

    if [ ${#matches[@]} -eq 1 ]; then
        echo "$matches[1]"
        return 0
    fi

    if [ ${#matches[@]} -eq 0 ]; then
        return 1
    fi

    echo "MULTIPLE"
    return 2
}

find_tool() {
    local candidate
    for candidate in "$@"; do
        if [ -n "$candidate" ] && [ -f "$candidate" ]; then
            echo "$candidate"
            return 0
        fi
    done
    return 1
}

HEX_SOURCE="$(find_hex_file)"
HEX_RESULT=$?

if [ $HEX_RESULT -eq 1 ]; then
    echo "No HEX file found in:"
    echo "$SCRIPT_DIR"
    osascript -e 'display dialog "No HEX file was found in the same folder as t.command." buttons {"OK"} default button "OK"'
    exit 1
fi

if [ $HEX_RESULT -eq 2 ] || [ "$HEX_SOURCE" = "MULTIPLE" ]; then
    echo "More than one HEX file was found in:"
    echo "$SCRIPT_DIR"
    echo
    echo "Please leave only one HEX file in this folder, or rename the wanted one to TX.HEX"
    osascript -e 'display dialog "More than one HEX file was found. Please leave only one HEX file in the folder, or rename the wanted one to TX.HEX." buttons {"OK"} default button "OK"'
    exit 1
fi

LOADER="$(find_tool \
    "$HOME/.platformio/packages/tool-teensy/teensy_loader_cli" \
    "/Applications/Teensy.app/Contents/Java/hardware/tools/teensy_loader_cli" \
    "/Applications/Teensyduino.app/Contents/Java/hardware/tools/teensy_loader_cli" \
    "$(command -v teensy_loader_cli 2>/dev/null)"
)"

REBOOT="$(find_tool \
    "$HOME/.platformio/packages/tool-teensy/teensy_reboot" \
    "/Applications/Teensy.app/Contents/Java/hardware/tools/teensy_reboot" \
    "/Applications/Teensyduino.app/Contents/Java/hardware/tools/teensy_reboot" \
    "$(command -v teensy_reboot 2>/dev/null)"
)"

echo "Using HEX file:"
echo "$HEX_SOURCE"
echo
echo "Log file:"
echo "$LOG"
echo

if [ -z "$LOADER" ] || [ ! -x "$LOADER" ]; then
    echo "Could not find teensy_loader_cli"
    osascript -e 'display dialog "Could not find teensy_loader_cli." buttons {"OK"} default button "OK"'
    exit 1
fi

echo "Found teensy_loader_cli:"
echo "$LOADER"
echo

if [ -n "$REBOOT" ] && [ -x "$REBOOT" ]; then
    echo "Found teensy_reboot:"
    echo "$REBOOT"
else
    echo "teensy_reboot not found. Reboot request will be skipped."
    REBOOT=""
fi

echo
echo "========================================"
echo "ATTEMPT 1"
echo "========================================"
echo

if [ -n "$REBOOT" ]; then
    "$REBOOT" >/dev/null 2>&1
    sleep 2
fi

"$LOADER" --mcu=imxrt1062 -w -s -v "$HEX_SOURCE" > >(tee "$LOG") 2>&1
RESULT1=$?

echo
echo "Attempt 1 exit code = $RESULT1"
echo

if [ $RESULT1 -eq 0 ]; then
    echo "Done. The HEX file has been loaded into the Teensy 4.1."
    osascript -e 'display dialog "Done. The HEX file has been loaded into the Teensy 4.1." buttons {"OK"} default button "OK"'
    exit 0
fi

echo "========================================"
echo "ATTEMPT 2"
echo "========================================"
echo

if [ -n "$REBOOT" ]; then
    "$REBOOT" >/dev/null 2>&1
    sleep 2
fi

"$LOADER" --mcu=imxrt1062 -w -s -v "$HEX_SOURCE" > >(tee -a "$LOG") 2>&1
RESULT2=$?

echo
echo "Attempt 2 exit code = $RESULT2"
echo

if [ $RESULT2 -eq 0 ]; then
    echo "Done. The HEX file has been loaded into the Teensy 4.1."
    osascript -e 'display dialog "Done. The HEX file has been loaded into the Teensy 4.1." buttons {"OK"} default button "OK"'
    exit 0
fi

echo "Upload failed twice."
echo "Please press the PROGRAM button on the Teensy, then run t.command again."
osascript -e 'display dialog "Upload failed twice. Please press the PROGRAM button on the Teensy, then run t.command again." buttons {"OK"} default button "OK"'

exit $RESULT2