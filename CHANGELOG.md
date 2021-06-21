## v2.1.0

### Changed
 - Custom protocol button messages
 - Button read algoritm rewritten, reducing crosstalk

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
