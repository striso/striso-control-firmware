# Striso Control Firmware

This repository contains the firmware source code for the Striso board MPE MIDI controller. More information can be found on www.striso.org.

## Updating firmware

Plug in the Striso board with the config (square) button pressed, now it should attach as a flash drive. To update the firmware just copy the new .uf2 file to this drive, after which the device automatically reboots. The current firmware version can be found in FW_INFO.TXT, and can be backupped by copying CURRENT.UF2.

The UF2 bootloader can be found on https://github.com/striso/uf2-stm32f.

## Requirements

- ChibiOS 2.6, should be located in ../ChibiOS_2.6
- gcc-arm-none-eabi
- git (for version number)
- python (for uf2 file creation)
