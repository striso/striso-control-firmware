# Changelog

## Next release

### Added
- Transpose reset with settings+up+down buttons
- Irregular tuning support, JI 7-limit on settings + A#3

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
