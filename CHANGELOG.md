# Changelog

## Next release

### Added

- Free transpose with up+down buttons pressed

### Changed
- Include number in product string to distinguish MIDI devices when multiple Striso boards are used

## v2.1.0 - 2021-06-21

This release only works well with the new hardware revision (#141 and up).

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
