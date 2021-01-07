#!/bin/bash
cnt=$(grep -c 'Driver=usbfs' /sys/kernel/debug/usb/devices)
if [ $cnt -gt 0 ]; then
        ExecStartPre=/sys/bus/usb/devices/1-1.2:1.0/driver/ "echo 1-1.2:1.0 > unbind"
        echo "Removed linux usbfs binding"
else
	echo "Linux usbfs binding does not exist"
fi
