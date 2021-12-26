## ChibiOS I2C test for Raspberry Pi Zero ##

### What it is ###

This project serves as a starting point for writing code on the Raspberry Pi Zero using
[Steve Bate's](https://www.stevebate.net/chibios-rpi/GettingStarted.html)
[ChibiOS-RPi](https://github.com/steve-bate/ChibiOS-RPi) fork of [ChibiOS](https://github.com/mabl/ChibiOS).

The goal is to retrieve pressure, temperature, and humidity readings from
sensors such as the MS8607 (PHT) and MS5840 (PT) (both from TE Connectivity).

### How to build ###

**Prerequisite : arm-none-eabi toolchain**

You'll need an `arm-none-eabi` toolchain built with `--with-cpu=arm1176jzf-s --with-fpu=vfp --with-float=hard`.
([This page](https://forums.raspberrypi.com/viewtopic.php?t=225731) was helpful
in figuring out why typical ARM toolchains were no good for the RPi Zero, and to know what challenges needed solving.)

At the start of this project, it was difficult to find a modern, ready-made toolchain like that, especially
if path-indepedence is one of your pet-peeves.
So I made [this one](https://drive.google.com/file/d/1Cn6uXWjJw1NNnBi1Az99kEKoWq8GaSdi/view?usp=sharing)
using [crosstool-ng](https://crosstool-ng.github.io/).
Due to health issues and related time constraints, I didn't get to feature-creep
it as much as I'd like. It's also only built for Linux (x86_64) right now. But
it does have a working GCC cross-compiler (version 11.2.0) and relevant
binutils+runtimes+etc. And it should work from any directory you choose to place
it in (in other words, it is path-independent); all you need to do is make sure
that the `<toolchain-path>/bin` path is in your *PATH* environment variable.

**Prerequisite : FAT32 formatted microSD card**

The executable produced by this project will need to be placed onto a microSD
card that is inserted into the Raspberry Pi Zero. Regarding the partitioning
and formatting of this card, here's what worked for me:

* The Raspberry Pi Zero's microSD card should contain a DOS partition table
with a single partition having type "W95 FAT32 (LBA)" (hex code `c` in `fdisk`).

* The single partition should be formatted with a vfat (FAT32) filesystem.
Note that `mkfs.vfat` has a `-n <NAME>` option that can be used to label (name)
the filesystem, which can be helpful for identifying the thing.

**Prerequisite : MS8607 or MS5840 on I2C pins**

This code establishes I2C communication with either an MS8607 or MS5840
sensor connected to the Raspberry Pi Zero's I2C pins (GPIO 2 and GPIO 3).

TODO: I intend to also write code that allows 2 of these sensors to be
connected to the same I2C pins by using a separate GPIO pin (or pins) to
select which one is being communicated with.


**Build and install**

```
git clone https://github.com/chadjoan/cdj-testing-rpzero-i2c.git
cd cdj-testing-rpzero-i2c
make
```

It should mention the creation of the `build/sdcard-final-contents` directory.
This directory has all of the files that need to end up on the microSD card.
So the next step simply involves plugging in the microSD card and copying
the contents of `build/sdcard-final-contents` onto the microSD card:
```
cp -r build/sdcard-final-contents/* /path/to/sd-card/
```

After that, just unmount (eject) and remove the microSD card, put it into the
Raspberry Pi Zero, plug in (turn on) the RPi Zero, then (hopefully) enjoy some
successful and happy blinkenlight.

