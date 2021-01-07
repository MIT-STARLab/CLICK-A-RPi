#!/bin/bash
if grep -c 'Driver=(none)' /sys/kernel/debug/usb/devices
then
        echo "Linux usbfs binding does not exist"
else
	ExecStartPre=/sys/bus/usb/devices/1-1.2:1.0/driver/ "echo 1-1.2:1.0 > unbind"
        echo "Removed linux usbfs binding"
fi
