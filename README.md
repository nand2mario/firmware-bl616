# TangCores Companion 

This is the WIP companion firmware for the on-board BL616 of Tang Console.

What is working and not working.
* (OK) USB host using the BL616 USB-C port.
* (to-be-integrated) JTAG programming of FPGA bitstream
* (todo) Menu and ROM loading
* (todo) 

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

For the USB drive, You need make an adapter to turn the connector from a "device" one to a "host" one.

<a href="doc/usbdrive.jpg"><img src="doc/usbdrive.jpg" width=400 /></a>

Here the orange wire provides 5V power to the USB VBUS pin. And the GND, D+ and D- pins are connected normally.

