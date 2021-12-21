## ChibiOS test LED blinker for Raspberry Pi Zero ##

### What it is ###

This project serves as a starting point for writing code on the Raspberry Pi Zero using
[Steve Bate's](https://www.stevebate.net/chibios-rpi/GettingStarted.html)
[ChibiOS-RPi](https://github.com/steve-bate/ChibiOS-RPi) fork of [ChibiOS](https://github.com/mabl/ChibiOS).
All it does is repeatedly pull GPIO pin 16 high for 900 milliseconds and low for 100 milliseconds.

It's pretty much the demo from the [`./demos/ARM11-BCM2835-GCC`](https://github.com/steve-bate/ChibiOS-RPi/tree/master/demos/ARM11-BCM2835-GCC)
directory of [ChibiOS-RPi](https://github.com/steve-bate/ChibiOS-RPi),
but with some minor changes (config, style, build, etc) and a minimal set of files
that can be placed on the RPi Zero's SD card to make it boot.

### Why it is ###

The intent here is to get bare-metal C code (no Linux) to work on the Raspberry Pi Zero
while still having access to a hardware abstraction layer (as provided by ChibiOS)
as well as a simplistic thread scheduler (also from ChibiOS). This is all with the
aim of having highly predictable behavior, and hopefully enjoying some nice
side-benefits like fast boot times and vastly fewer dependencies. Those are the
technical goals, at least.

This started as the first step towards having a device that can, with a closed-loop
control system, pressurize one's ears during CPAP usage. It would thus provide
a way to avoid ear pain and injury while using CPAP at whatever pressure settings
are most effective at preventing apnea events, regardless of the genetic makeup
of one's ears. This task may someday involve various types of signals processing,
such as active noise cancellation (ANC) and breathing pattern prediction, that
may greatly benefit from the Pi Zero's whopping 512MB of RAM. (As of this writing,
high-end STM32 offerings like the [STM32H7A3](https://www.st.com/content/st_com/en/products/microcontrollers-microprocessors/stm32-32-bit-arm-cortex-mcus/stm32-high-performance-mcus/stm32h7-series/stm32h7a3-7b3.html)
max out at about 1.4MB of RAM, with a lot of that dissappearing during its sleep mode,
which means I'd have to worry about memory depletion and activation statefulness
while writing code, which sounds kinda awful. The RPi Zero is generally cheaper, too.
The STM32 would no doubt completely dominate at power consumption, but that's
not an important criteria for something that's going to be generating air
pressure and possibly operating motors to open and close valves.)

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

**Prerequisite : LED**

This code sets and clears GPIO pin number 16 on the RPi Zero. The RPi Zero
does NOT have a LED connected to this pin. (As far as I know, it doesn't have
*any* LEDs connected to the GPIO pins.)

The intent of this test|example project is to perform some visible action
that proves the C code is working. A simple way to do this would be to
connect a LED to pin 16 (more specifically: connect `pin16 -> LED -> resistor -> GND`).
[Always put a resistor in series with the LED](https://thepihut.com/blogs/raspberry-pi-tutorials/27968772-turning-on-an-led-with-your-raspberry-pis-gpio-pins),
as LEDs can draw a lot of current on their own, which would destroy the LED and/or the Raspberry Pi Zero.

Before even bothering to solder/crimp/connect/etc a LED and resistor by hand,
I highly recommend purchasing a breakout board for the Raspberry Pi, such as
[this one](https://amzn.com/dp/B08RDYDG6X) that (at the time of this writing)
costs about $20 USD. Such a board provides LEDs for every GPIO pin on the
RPi Zero, which makes it possible to easily visualize what the code is doing.
For example, the function call `palSetPad(ONBOARD_LED_PORT, ONBOARD_LED_PAD);`
doesn't indicate "GPIO 16" anywhere. To make that connection, one would need
to dig into ChibiOS-RPi's source code, figure out what it's doing, possibly
correlate numbers between different pin numberings, and definitely don't make
any mistakes during any of the logical deductions along the way.
It's doable, but momentarily glancing over at a grid of neatly arranged
surface-mount LEDs is *so much* easier. The aforementioned breakout board also
provides screw terminals for every GPIO pin (and the power+ground pins),
which makes it easy to quickly and securely attach test components or modules/boards.

Of course, there could be easier-but-less-rewarding ways to accomplish this,
such as hooking a multimeter up to GPIO #16 and watching for the voltage
changes.

**Build and install**

```
git clone https://github.com/chadjoan/cdj-testing-rpzero-blinker.git
cd cdj-testing-rpzero-blinker
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

