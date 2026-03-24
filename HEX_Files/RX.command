#!/bin/zsh
setopt NULL_GLOB

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
LOG="$SCRIPT_DIR/rx_upload_log.txt"

echo "========================================"
echo "UPLOAD HEX TO RECEIVER TEENSY 4.0"
echo "========================================"
echo
echo "Running from:"
echo "$SCRIPT_DIR"
echo

find_tool() {
    local candidate
    for candidate in "$@"; do
        if [ -f "$candidate" ]; then
            echo "$candidate"
            return 0
        fi
    done
    return 1
}

find_hex_file() {
    local matches=()

    # First, prefer likely RX names
    for f in \
        "$SCRIPT_DIR/RX.HEX" \
        "$SCRIPT_DIR/RX.hex" \
        "$SCRIPT_DIR/Rx.HEX" \
        "$SCRIPT_DIR/Rx.hex" \
        "$SCRIPT_DIR/rx.HEX" \
        "$SCRIPT_DIR/rx.hex"
    do
        if [ -f "$f" ]; then
            echo "$f"
            return 0
        fi
    done

    # Otherwise accept any single .hex/.HEX file in the folder
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

HEX_SOURCE="$(find_hex_file)"
HEX_RESULT=$?

if [ $HEX_RESULT -eq 1 ]; then
    echo "No HEX file found in:"
    echo "$SCRIPT_DIR"
    osascript -e 'display dialog "No HEX file was found in the same folder as r.command." buttons {"OK"} default button "OK"'
    exit 1
fi

if [ $HEX_RESULT -eq 2 ] || [ "$HEX_SOURCE" = "MULTIPLE" ]; then
    echo "More than one HEX file was found in:"
    echo "$SCRIPT_DIR"
    echo
    echo "Please leave only one HEX file in this folder, or rename the wanted one to RX.HEX"
    osascript -e 'display dialog "More than one HEX file was found. Please leave only one HEX file in the folder, or rename the wanted one to RX.HEX." buttons {"OK"} default button "OK"'
    exit 1
fi

LOADER="$(find_tool \
    "$HOME/.platformio/packages/tool-teensy/teensy_loader_cli" \
    "$HOME"/.platformio/packages/tool-teensy@*/teensy_loader_cli \
    "$HOME"/Library/Arduino15/packages/teensy/tools/teensy-tools/*/teensy_loader_cli \
    "/Applications/Teensy.app/Contents/Java/hardware/tools/teensy_loader_cli" \
    "/Applications/Teensyduino.app/Contents/Java/hardware/tools/teensy_loader_cli" \
    "/usr/local/bin/teensy_loader_cli" \
    "/opt/homebrew/bin/teensy_loader_cli"
)"

REBOOT="$(find_tool \
    "$HOME/.platformio/packages/tool-teensy/teensy_reboot" \
    "$HOME"/.platformio/packages/tool-teensy@*/teensy_reboot \
    "$HOME"/Library/Arduino15/packages/teensy/tools/teensy-tools/*/teensy_reboot \
    "/Applications/Teensy.app/Contents/Java/hardware/tools/teensy_reboot" \
    "/Applications/Teensyduino.app/Contents/Java/hardware/tools/teensy_reboot" \
    "/usr/local/bin/teensy_reboot" \
    "/opt/homebrew/bin/teensy_reboot"
)"

# If not found above, try a broader search
if [ -z "$LOADER" ] || [ ! -f "$LOADER" ]; then
    LOADER="$(find "$HOME/.platformio" "$HOME/Library/Arduino15" /Applications /usr/local/bin /opt/homebrew/bin \
        -name teensy_loader_cli 2>/dev/null | head -n 1)"
fi

if [ -z "$REBOOT" ] || [ ! -f "$REBOOT" ]; then
    REBOOT="$(find "$HOME/.platformio" "$HOME/Library/Arduino15" /Applications /usr/local/bin /opt/homebrew/bin \
        -name teensy_reboot 2>/dev/null | head -n 1)"
fi

echo "Using HEX file:"
echo "$HEX_SOURCE"
echo
echo "Log file:"
echo "$LOG"
echo

if [ -n "$LOADER" ] && [ -f "$LOADER" ]; then
    echo "Found teensy_loader_cli:"
    echo "$LOADER"
else
    echo "Could not find teensy_loader_cli"
    osascript -e 'display dialog "Could not find teensy_loader_cli. Please install Teensy tools or PlatformIO first." buttons {"OK"} default button "OK"'
    exit 1
fi

echo
if [ -n "$REBOOT" ] && [ -f "$REBOOT" ]; then
    echo "Found teensy_reboot:"
    echo "$REBOOT"
else
    echo "teensy_reboot not found. Auto-reboot request will be skipped."
fi

echo
echo "========================================"
echo "ATTEMPT 1: ASK RUNNING CODE TO REBOOT"
echo "========================================"
echo

if [ -n "$REBOOT" ] && [ -f "$REBOOT" ]; then
    "$REBOOT" >/dev/null 2>&1
    sleep 2
else
    echo "Skipping reboot request because teensy_reboot was not found."
    echo
fi

echo "========================================"
echo "ATTEMPT 2: DIRECT LOADER, SHORT WAIT"
echo "========================================"
echo

echo "If the receiver does not enter bootloader by itself,"
echo "you will need to press the PROGRAM button."
echo

"$LOADER" --mcu=imxrt1062 -w -s -v "$HEX_SOURCE" | tee "$LOG" &
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
    echo "Done. The HEX file has been loaded into the Teensy 4.0."
    osascript -e 'display dialog "Done. The HEX file has been loaded into the Teensy 4.0." buttons {"OK"} default button "OK"'
else
    echo "Upload failed."
    echo "This receiver probably cannot auto-reboot into programming mode."
    osascript -e 'display dialog "Upload failed. This receiver probably cannot auto-reboot into programming mode, so the PROGRAM button is needed." buttons {"OK"} default button "OK"'
fi

exit $LOADER_RESULT