[![latest release](https://img.shields.io/github/v/release/striso/striso-control-firmware)](https://github.com/striso/striso-control-firmware/releases/latest)

# Striso Control Firmware

This repository contains the firmware source code for the Striso board MPE MIDI controller. More information can be found on www.striso.org.

## Updating firmware

The latest release can be found on https://github.com/striso/striso-control-firmware/releases/latest.

To update the Striso board firmware plug in the Striso board with the config (square) button pressed, now it should attach as a flash drive. To update the firmware just copy the new .uf2 file to this drive, after which the device automatically reboots. The current firmware version can be found in INFO_FW.TXT, and can be backupped by copying CURRENT.UF2.

The UF2 bootloader can be found on https://github.com/striso/uf2-ChibiOS.

## Build requirements

- gcc-arm-none-eabi
- faust (when synthesizer is modified)
- git (for version number)
- python3 (for uf2 file creation)
