#!/bin/bash
cnt=$(grep -c 'Driver=usbfs' /sys/kernel/debug/usb/devices)
if [ $cnt -eq 0 ]; then
        echo "Linux usbfs binding does not exist"
else
	ExecStartPre=/sys/bus/usb/devices/1-1.2:1.0/driver/ "echo 1-1.2:1.0 > unbind"
        echo "Removed linux usbfs binding"
fi
