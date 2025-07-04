# Changelog

## v2.2.2 - 2025-07-04

With this firmware release a preset can be selected to load on power on. This is useful in some scenario's, but may cause unexpected behaviour if you forget that you or someone else changed the default.

To load the default settings instead press the glissando button during power on.

### Added
- A preset can now be loaded at power on.
- Add option to disable settings button for simplified use (for SoundLAB). Then it controls pedal instead.
- Add option to switch settings and glissando button position
- Pressing the glissando button at power on now loads default settings
- Make sustain pedal decay values configurable in presets

### Removed
- Settings for aux jack and motion sensor in 'General'. A preset should now be used instead.

### Changed
- Semitone transpose is now dependent on tuning, and transposes by a minor second
- Take into account per note offsets in MIDI note mode
- Send motion over serial MIDI too. Since sending motion data is disabled by default it shouldn't cause problems

## v2.2.1 - 2023-04-05

This firmware release finally brings pedal support to the Striso board!
With that also comes an extra control for the builtin synth: sustain/decay time, which can also be set with [settings] + Ab1 or by sending CC64 (continuous).

By default the jack autodetects what is plugged in. It detect the following possibilities:
- Cyan blink: MIDI out (TRS Type A)
- Yellow blink: Expression pedal, TRS/wiper-on-tip type
- Pink blink: Single/double switch pedal. Triple switch support will be added later.
- Red blinking: Error, either wrong type MIDI or expression pedal or other issue. Unplug to reset.

If autodetection doesn't work well for some reason the connection can be explicitly configured in a preset.
Additionally the jack can be used as line-in to mix other audio (for example a second Striso board) with the audio output. For this you can make a preset.

For now the pedal functions are fixed:
- Expresssion pedal controls the sustain/decay time.
- Switch pedal 1/Sustain (tip signal) toggles sustain between 32 and 96.
- Switch pedal 2 (ring signal) controls the glissando mode.

In addition three user feature requests are implemented, see below for all changes.

Feedback about if the jack detection works well is very welcome!

### Added
- Pedal support!
- Add MIDI velocity offset setting to make it possible to send constant velocity notes
- Settings up/down autorepeat (with dynamic repeat rate!)
- Blink pink light on MIDI receive

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
