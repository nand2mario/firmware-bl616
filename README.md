# TangCore bl616 firmware 

This is the WIP firmware for the on-board BL616 of Tang Console.

What is working and not working.
* (OK) USB host using the BL616 USB-C port.
* (OK) JTAG programming of FPGA bitstream
* (OK) Menu and ROM loading
* (todo) Core status check and display refresh
* (todo) Core and rom loading in a single step

This needs Harbaum's patched version of Bouffalo SDK.

```
cd ..
git clone --recursive https://github.com/harbaum/bouffalo_sdk.git
```

Then it should build OK.

```
make
make flash COMX=com5
```

The 2nd line flashes the firmware to BL616. Before executing that, press and hold the "BOOT" button on the Tang Console board (bottom left corner, close to one of the USB-C port), then plug in the USB cable. This enters the BL616 into the programming mode.

For the USB drive, You need use an OTG adapter to turn the connector from a "device" one to a "host" one.

<a href="doc/usbdrive.jpg"><img src="https://github.com/nand2mario/tangcores/blob/main/doc/tangcores-dev-setup.jpg?raw=true" width=400 /></a>


