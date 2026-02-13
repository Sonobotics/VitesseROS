#!/bin/bash
if lsmod | grep -q '^ftdi_sio'; then
    for dev in /sys/bus/usb/drivers/ftdi_sio/*:*; do
        if [[ -e "$dev" ]]; then
            devname=$(basename "$dev")
            echo "$devname" | sudo tee /sys/bus/usb/drivers/ftdi_sio/unbind > /dev/null
        fi
    done
fi
