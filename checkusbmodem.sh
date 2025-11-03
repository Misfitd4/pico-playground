!/bin/bash

DEVICE_PATTERN="/dev/tty.usbmodem*"

while true; do
    if ls $DEVICE_PATTERN 1> /dev/null 2>&1; then
        echo "Device found: $(ls $DEVICE_PATTERN)"
    else
        echo "Device not found"
    fi
    sleep 1
done
