# WAV Access Controller

ESP32-S3 firmware for the WAV door, ramp, and air-suspension controller.

## Current firmware

`WAV_Controller_R2_1_2026-10-11.ino`

- After the door closes, suspension raising waits for ignition.
- Ignition must remain on for 20 seconds before raising and balancing.
- Bag balancing has a 15-second timeout.
- After 20 minutes with ignition off, a pending post-exit raise is cancelled and the controller enters deep sleep.
- Entry, exit, and ignition inputs wake the controller.
- The firmware revision is shown at the top of the source and at the bottom of the controller web page.

## Target

- ESP32-S3
- Arduino ESP32 core 3.3.8

Bench-test all input polarity, relay outputs, limit values, and pneumatic safety behaviour before operating the installed system.
