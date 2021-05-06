# Striso development utilities

Utilities to help with Striso firmware development.

## Requirements

Works on Linux.

- liblo-dev
- libusb-1.0-0-dev
- faust (for synthesizer)
- libjack-jackd2-dev (or alternatively jackd1 or compile synth for other target)

Build with `make`, this compiles the utilities, synthesizer, and updates the USB
permissions to allow connection to the Striso binary protocol (needs sudo rights).

Launch the synth and OSC bridge with `make launch_synth`.

## Utilities

`striso_util`: low level Striso communication utility

`stribri.sh`: read data from Striso and send as OpenSoundControl on port 5510

`synth`: synthesizer, uses JACK for audio

`strivi.py`: Striso data visualisation utility
