# TangCore firmware for BL616  

This is TangCore firmware for the on-board BL616 of Tang Console.

See [this document](https://github.com/nand2mario/tangcore/blob/main/doc/dev.md) for how the firmware works with cores.

This needs Vossstef's patched version of Bouffalo SDK.

```
cd ..
git clone --recurse-submodules https://github.com/vossstef/bouffalo_sdk.git
```

Then it should build OK.

```
make
make flash COMX=com5
```

The 2nd line flashes the firmware to BL616. Before executing that, press and hold the "BOOT" button on the Tang Console board (bottom left corner, close to one of the USB-C port), then plug in the USB cable. This enters the BL616 into the programming mode.

For the USB drive, You need an OTG dongle to turn the connector from a "device" one to a "host" one, and provide power at the same time.

Acknowledgements
* JTAG FPGA programming logic based on [openFPGALoader](https://github.com/trabucayre/openFPGALoader)
* Gamepad support based on Till Harbaum's [FPGA-Companion](https://github.com/harbaum/FPGA-Companion)
