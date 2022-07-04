# Changelog

## Next version

### Added
- Retrigger delay, no note on message will be sent within 50ms after a note off message.

### Changed
- Config editor tweaks

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
