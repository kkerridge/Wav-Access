# WAV Access Controller

Replacement control software for the failed access-system electronics in a Sirus-converted Volkswagen Caddy 3 wheelchair-accessible vehicle (WAV).

The project retains the vehicle's existing pneumatic system, actuators, valves, and analogue sensors. The original electronic controller is replaced by an ESP32-S3 Waveshare relay board with an added external ADC for reading the sensor signals. The software coordinates the rear door, access ramp, compressor, air supply, exhaust, and left and right suspension bags from the vehicle inputs or a local web interface.

## Project purpose

The original Sirus conversion electronics failed, leaving the pneumatic access equipment without a working controller. This firmware provides a serviceable replacement control system using readily available hardware. It is intended specifically for the converted VW Caddy 3 being repaired and reproduces the required entry, exit, suspension-lowering, suspension-raising, and balancing operations.

## Replacement hardware

- ESP32-S3 Waveshare relay board as the main controller.
- Eight relay outputs for the compressor, exhaust, air supply, left bag, right bag, ramp, door-open, and door-close circuits.
- External ADS1115 ADC connected over I2C.
- Four analogue measurements: left suspension height, right suspension height, tank pressure, and door position.
- Physical stop, entry/open, exit/close, and ignition inputs.
- The vehicle's existing pneumatic equipment and sensors.

## What the software does

### Entry sequence

Entry is permitted only while the ignition is off. When an entry/open signal is received, the controller:

1. Exhausts the left and right suspension bags until their calibrated dump positions are reached.
2. Opens the rear door.
3. Briefly applies the opposite door valve at the calibrated slow-open point to soften the final part of the movement.
4. Deploys the access ramp.

The ignition input acts as an interlock. Switching the ignition on during a door-opening sequence stops the sequence and turns off the relay outputs.

### Exit sequence

When an exit/close signal is received, the controller:

1. Stows the access ramp.
2. Closes the rear door to its calibrated closed position.
3. Leaves the suspension lowered while waiting for the driver to switch on the ignition.
4. Requires the ignition to remain on continuously for 20 seconds before operating the compressor and suspension valves.
5. Raises each suspension bag independently to its calibrated working height.
6. Allows the bags to settle, then tops up either side as required to balance the vehicle.

If ignition is removed during the delay, raising, or balancing, the outputs stop and the controller returns to waiting for ignition. If ignition is not switched on within 20 minutes, the pending raise is treated as abandoned and is cancelled before the controller sleeps.

### Suspension monitoring

While the ignition remains on, the software periodically checks the suspension. It samples both height sensors to make sure the vehicle is stable before deciding whether a top-up is required. A low, stable side is raised to its saved working height and the normal balancing routine is then run.

The compressor is used only when air is required and the measured tank pressure is below the configured level. Left and right bag valves are controlled separately so one side can finish before the other.

### Low-power operation

After the ignition has been off for 20 minutes, the controller stops all relays, shuts down Wi-Fi, and enters ESP32 deep sleep. Entry, exit, and ignition inputs are configured as wake sources. Short entry or exit pulses that caused a wake are captured during startup so the requested sequence is not lost while the ESP32 reboots.

### Web interface and calibration

The ESP32 creates a local Wi-Fi access point and serves a browser-based control page at 192.168.4.1  . The page provides:

- Automatic entry, automatic exit, and stop controls.
- Live tank-pressure, sensor, and operating-state information.
- Manual jog controls for the door, suspension bags, and air tank.
- Calibration controls for bag dump height, bag working height, tank pressure, door-open position, door-closed position, and the slow-open point.
- Persistent storage of calibration settings in ESP32 Preferences memory.

The firmware revision is displayed at the top of the source file and at the bottom of the web page.

### Safety behaviour

- The STOP input and web STOP command immediately switch off all relay outputs.
- Manual jog commands stop automatically if command updates are lost.
- Door, ramp, suspension-raising, balancing, and top-up operations have time limits to prevent continuous operation following a sensor or mechanism fault.
- Bag balancing has a 15-second timeout.
- The system stops suspension raising if ignition is removed.
- Relay outputs are turned off before entering deep sleep.

## Current firmware

`WAV_Controller_R2_1_2026-10-11.ino`

- Firmware revision: `WAV-R2.1-2026-10-11`
- After the door closes, suspension raising waits for ignition plus 20 seconds.
- The pending post-exit raise is cancelled when the 20-minute low-power timeout expires.

## Target

- Sirus-converted Volkswagen Caddy 3 WAV
- ESP32-S3
- Waveshare relay controller with external ADS1115 ADC
- Arduino ESP32 core 3.3.8

Bench-test all input polarity, relay outputs, limit values, and pneumatic safety behaviour before operating the installed system.
