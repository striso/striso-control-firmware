# Changelog

## next release

### Added
- Add MIDI velocity offset setting to make it possible to send constant velocity notes
- Settings up/down autorepeat (with dynamic repeat rate!)

### Changed
- Invert up-down buttons and leds when layout is inverted

## v2.2.0 - 2022-07-21

With this release the way settings work has drastically changed to allow more flexible settings and give better feedback.
More settings are added and the existing ones have more precise control.

The new system uses one button per setting, and uses the up/down leds to show the current value.
The value of a setting can be changed by nudging the key up or down.
Larger changes can be made by making a circular motion clockwise (up) or anticlockwise (down) with the key.

See the updated quick start guide for more information and an updated settings overview.

### Added
- On board settings for MPE voice count, midi channel, tuning offset, MPE pitch bend range
- Volume parameter in presets, and listen to MIDI CC7 - Volume
- Send MPE and pitch bend range RPN messages on start, mode change, and send cfg key
- Additional tuning offset parameter in presets

### Changed
- New settings system and layout
- Configuration editor tweaks
- Response to MIDI CC 126 Mono mode and 127 Poly mode changed
- MIDI Mono mode on channel 2 by default to be more compatible with MPE
- Send MIDI Program Change on all used channels for more compatibility with non MPE synths

## v2.1.5 - 2022-07-01

This release adds a long planned feature: Persistent configuration and a configuration editor.

The editor can be opened on a computer. Connect the USB cable with the settings (square) button pressed, now it should attach as a flash drive. Open the file CONFIG.HTM, which should open a web browser with the configuration editor.

Saving the configuration doesn't work with the Safari browser, please use another browser for now.

### Added
- Persistent configuration and configuration editor
- MIDI note modes, to have each key send a unique note number (accessible through the configuration editor)

### Changed
- Don't send motion sensor data over MIDI by default, this behaviour can be changed in the config editor
- Dim up/down leds when the note shift is less than a whole octave

## v2.1.4 - 2021-12-03

### Added
- Setting for key detection threshold, to stop false MIDI note-on messages

### Changed
- Merge 31TET and meantone tunings, keep both Bohlen-Pierce mappings

## v2.1.3 - 2021-12-02

### Added
- Irregular tuning support, JI 7-limit on settings + A#3
- Transpose reset with settings+up+down buttons

### Changed
- Don't decrease detected zero levels of keys that trigger erroneously.
  They were allowed to go down, which caused erroneous note on messages.
  This also means that a key will be less sensitive if it's held down during start up.

### Fixed
- Correctly lower message frequency when multiple keys are pressed (was too low in some cases)

## v2.1.2 - 2021-10-28

### Changed
- Red led when transpose is out of valid range

### Fixed
- MIDI semitone offset (introduced in v2.1.1)
- Free transpose improvements

## v2.1.1 - 2021-10-20

This release brings the new button read algoritm to rev0 hardware (white aux buttons).

With the rev0 hardware the signals are a bit more noisy, the button crosstalk
is much reduced compared to the v2.0.x firmware though. Feedback welcome!

### Added
- Free transpose with up+down buttons pressed. With this mode the pitch of one button can be transfered to another button.
- Option to flip layout (rotate 180 degrees) with settings + B4
- Support for rev0 hardware

### Changed
- Include number in product string to distinguish MIDI devices when multiple Striso boards are used
- Stuck note detection improved (dynamic zero level detection). Feedback welcome!

## v2.1.0 - 2021-06-21

v2.1.x features an improved button read algoritm, hardware is updated to rev1 for improved button sensitivity.

This release only works well with the new hardware revision (rev1, #141 and up, black aux buttons).

### Changed
- Custom protocol button messages new format
- Button read algoritm rewritten, reducing crosstalk and lifting the four corner limitation
- Hardware revision: ADC resistor values changed for better sensitivity and less cross talk

### Fixed
- Clamp pres during portamento and fix portamento for very light presses

## v2.0.5 - 2021-05-31

### Added
- Bohlen-Pierce tuning (two mappings)
- Optional crosstalk filter (for testing)

### Fixed
- Portamento was not working in some situations

## v2.0.4 - 2021-05-14

### Added
- TRS MIDI support (no motion sensor data to reduce data rate)
- Normal MIDI mode: add pitch bend, tilt and channel pressure output
- Configurable CC for key press, bend and tilt
- Development utilities

### Fixed
- Enable line-out directly at boot to reduce ground-loop noise with some amps

---

For earlier releases check the commit logs.
