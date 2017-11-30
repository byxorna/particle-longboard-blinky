# particle-longboard-blinky
Particle Photon arduino code for longboard LEDs

## Setup

```
# get particle-cli
$ yarn install
```

## Compile

```
$ particle compile photon
attempting to compile firmware
downloading binary from: /v1/binaries/59e420266d43bc4ff5174c76
saving to: photon_firmware_1508122650655.bin
Memory use:
   text    data     bss     dec     hex filename
  15340     108    1588   17036    428c /workspace/target/workspace.elf

Compile succeeded.
Saved firmware to: /Users/gabe/code/particle-longboard-blinky/photon_firmware_1508122650655.bin
```

## Flash

```
$ particle flash --usb $(ls -t photon_firmware*bin|head -n1)
Found DFU device 2b04:d006
spawning dfu-util -d 2b04:d006 -a 0 -i 0 -s 0x080A0000:leave -D photon_firmware_1508122560924.bin
dfu-util 0.9

Copyright 2005-2009 Weston Schmidt, Harald Welte and OpenMoko Inc.
Copyright 2010-2016 Tormod Volden and Stefan Schmidt
This program is Free Software and has ABSOLUTELY NO WARRANTY
Please report bugs to http://sourceforge.net/p/dfu-util/tickets/

dfu-util: Invalid DFU suffix signature
dfu-util: A valid DFU suffix will be required in a future dfu-util release!!!
Opening DFU capable USB device...
ID 2b04:d006
Run-time device DFU version 011a
Claiming USB DFU Interface...
Setting Alternate Setting #0 ...
Determining device status: state = dfuIDLE, status = 0
dfuIDLE, continuing
DFU mode device DFU version 011a
Device returned transfer size 4096
DfuSe interface name: "Internal Flash   "
Downloading to address = 0x080a0000, size = 15448
Download        [=========================] 100%        15448 bytes
Download done.
File downloaded successfully

Flash success!
```

## Flashing over USB

To enter DFU Mode:
- Hold down BOTH buttons
- Release only the RST button, while holding down the MODE button.
- Wait for the LED to start flashing yellow
- Release the MODE button
- The Core now is in the DFU mode.

# TODO
- [ ] make pattern when flipped cylon eye
- [ ] make rainbow pattern for normal operation
- [ ] make disorient palette
