Installing driver
The makestuff directory needs to be set up from the tarball on this repo. 

The link provided in the documentation for libfpgalink is broken and won't set up the makestuff directory, so you have to start with the tarball. 

Then you can follow the steps in the installation documentation located in the libfpgalink git repo. 

Prerequesites:
* sudo apt-get install g++
* sudo apt-get install libusb-1.0-0-dev 
* sudo apt-get install sdcc



All that's needed is to cd into makestuff/libs, run ../scripts/msget.sh makestuff/libfpgalink, and then once it's downloaded that library, cd libfpgalink, and run make deps. 

You might get an error that says two undefined references to libusb functions. 

This is because the camera drivers installed another libusb library that's unnecessary, run ldconfig -p | grep libusb to see the path to the camera's libusb reference (should be /opt/mvIMPACT_Acquire/lib/armhf), go to this directory and delete both the symbolic link to libusb and the actual libusb.so file. Then run make deps again and this should install all of the libfgpalink files. 

If you have problems runnning the driver because of linking problem, you need to add a config file to /etc/ld.so.conf.d that has the path to libfgpalink.so (which is in the libfpgalink subdirectory). 
