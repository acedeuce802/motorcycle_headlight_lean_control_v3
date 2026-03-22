# Motorcycle Adaptive Cornering Lights V3

Lean-angle-controlled bi-LED projector rotation system for motorcycles.
Uses a worm-drive stepper motor to physically rotate a bi-LED projector
in response to lean angle, keeping the beam pointed into corners.

## Hardware

| Component | Part | Notes |
|---|---|---|
| Microcontroller | Seeed Xiao ESP32-C6 | Or ESP32-C3-WROOM-02 on custom PCB |
| IMU | BMI160 breakout | Wired directly to D4/D5, addr 0x68 |
| I2C mux | TCA9548A breakout | addr 0x70 |
| Distance sensors | DFRobot SEN0590 ×2 | TCA channels 0 (left) and 1 (right) |
| Stepper driver | TMC2209 module | Standalone mode, 1/8 microstepping |
| Stepper motor | NEMA 8, 1.8°/step | 8HS11-0204S or similar |
| Worm drive | M1.5, 72:1 ratio | Brass worm shaft, PET-CF printed wheel |
| Limit switches | Micro switch ×2 | Active LOW, left and right travel stops |
| Bearings | 6816-2RS (front), 6704ZZ (worm support) | Projector rotation |
| Regulator | MP1584 module | 12V → 5V for ESP32 and TMC VIO |


## Pin Assignments (Xiao ESP32-C6)

| Pin | GPIO | Signal | Notes |
|---|---|---|---|
| D0 | IO1 | STEP | TMC2209 step pulse |
| D1 | IO2 | DIR | TMC2209 direction |
| D2 | IO3 | EN | TMC2209 enable (active LOW) |
| D3 | IO21 | — | Spare |
| D4 | IO22 | I2C SDA | TCA9548A + BMI160 |
| D5 | IO23 | I2C SCL | TCA9548A + BMI160 |
| D6 | IO16 | LIMIT LEFT | Micro switch, INPUT_PULLUP, active LOW |
| D7 | IO17 | LIMIT RIGHT | Micro switch, INPUT_PULLUP, active LOW |
| D8 | IO18 | SPEED | Hall effect pulse input, INPUT_PULLUP |
| 5V | — | Power in | From MP1584 (12V → 5V) |
| 3V3 | — | Power out | To TCA9548A VCC, BMI160 VIN |
| GND | — | Ground | Common ground |

## I2C Addresses

| Device | Address | Connection |
|---|---|---|
| TCA9548A | 0x70 | Direct to D4/D5 |
| BMI160 | 0x68 | Direct to D4/D5 (SA0 → GND) |
| SEN0590 left | 0x74 | Via TCA channel 0 |
| SEN0590 right | 0x74 | Via TCA channel 1 |

## Stepper Configuration

| Parameter | Value |
|---|---|
| Motor | NEMA 8, 1.8°/step (200 steps/rev) |
| Microstepping | 1/8 (MS1=GND, MS2=GND) |
| Worm ratio | 72:1 |
| Steps per degree | 320 steps/° |
| Full travel | ±45° (limited by physical limit switches) |
| Homing speed | 800µs/step |
| Normal speed | 200µs/step |
| VREF | ~0.28V (set for 0.2A motor current) |


## Passive Components

### TMC2209 power rail (12V → VM)
- 10Ω 1W resistor in series before VM
- 470µF 35V electrolytic from VM to GND
- 100nF ceramic from VM to GND (high frequency)
- P6KE15A TVS diode: cathode → 12V rail, anode → GND
- 100nF ceramic from VIO to GND

### TCA9548A downstream channels
- 4.7kΩ from SD0 to 3.3V
- 4.7kΩ from SC0 to 3.3V
- 4.7kΩ from SD1 to 3.3V
- 4.7kΩ from SC1 to 3.3V
- 100nF ceramic from VCC to GND
- Note: master SDA/SCL (D4/D5) and address pins already pulled up on TCA breakout

### BMI160
- 100nF ceramic from VIN to GND
- SA0 → GND (sets I2C address 0x68)
- CS/OCS → 3.3V via 10kΩ (ensures I2C mode, if not already pulled up on breakout)

### Limit switches
- 10kΩ from each signal pin to 3.3V (pull-up)
- 100nF ceramic from each signal pin to GND (debounce)

### Speed sensor
- 10kΩ from signal pin to 3.3V (pull-up, if hall effect sensor has open-collector output)
- 100nF ceramic from signal pin to GND (noise rejection)

## Wiring Notes

**Motor coil wires (A1/A2/B1/B2):** 0.5mm (20mil) trace or 20 AWG wire minimum.
**12V power rail:** 0.8mm (32mil) trace or 20 AWG wire minimum.
**Signal traces:** 0.2mm (8mil) minimum, standard signal routing.
**GND:** Use flood fill ground plane on at least one PCB layer.

**TMC2209 standalone mode pin tie-offs:**
- MS1 → GND (1/8 microstepping with MS2=GND)
- MS2 → GND
- CLK → GND (use internal clock)
- PDN/PDN_UART → leave disconnected or tie to GND via 100nF cap

## Homing Sequence

On every power-on the system runs a three-phase homing sequence before
accepting lean angle commands:

1. **Left sweep** — motor rotates toward left limit switch at homing speed.
   Triggers when switch goes LOW. Records this as step 0.
2. **Back-off** — retracts 100 steps away from left limit.
3. **Right sweep** — motor rotates toward right limit switch, counting steps.
   Triggers when switch goes LOW. Records total travel as `homingMaxSteps`.
4. **Back-off** — retracts 100 steps from right limit.
5. **Center** — moves to `homingMaxSteps / 2`. This becomes the 0° reference
   (straight ahead). `currentSteps` is re-zeroed here.

If either limit switch is not found within 50,000 steps the homing fails and
the stepper is disabled. Use `/api/rehome` from the web interface to retry.


## Lean Angle Sources

### Distance sensors (geometry-based)
Both SEN0590 sensors measure distance to the ground. The difference in
readings, divided by the sensor spacing, gives the lean angle via `atan2`.
Valid only when both sensors return readings within the configured min/max
distance window.

### BMI160 gyro (Reidel model)
Yaw rate from the gyro is combined with wheel speed to estimate lean angle:

```
lean = atan(speed_m_s × yaw_rate_rad_s / 9.81)
```

Active only when speed > 2 m/s. Requires calibration of gyro zero bias at
startup (bike must be stationary during the first ~1 second after power-on).

### Fusion
When both sources are available and speed > 2 m/s:
```
lean_fused = 0.3 × lean_distance + 0.7 × lean_IMU
```
Falls back to whichever source is available individually.

## Web Interface

Connect to the WiFi AP (default SSID: `CL-V3-XXXXXX`, password: `cornering123`)
then browse to `http://192.168.5.1` or `http://CL-V3-XXXXXX.local`.

| Page | URL | Description |
|---|---|---|
| Dashboard | `/` | Live lean angle, projector position bar, sensor readings |
| Calibration | `/calibrate` | Lean mapping, sensor geometry, IMU axis, speed sensor |
| Config | `/config` | WiFi, device name, sensor source enable/disable |
| Test | `/test` | Manual projector angle control, quick presets, re-home |
| OTA | `/update` | Web-based firmware upload (.bin from Arduino IDE) |

### API endpoints

| Endpoint | Method | Description |
|---|---|---|
| `/api/status` | GET | JSON: all live state values |
| `/api/calibrate` | POST | Save calibration settings |
| `/api/config` | POST | Save config and restart |
| `/api/test` | GET/POST | Set/release test mode angle |
| `/api/rehome` | GET | Trigger homing sequence |
| `/api/reset` | GET | Factory reset (clears NVS) |

## Configuration (Calibration Page)

| Setting | Description |
|---|---|
| Max lean angle | Lean angle (°) that maps to full projector travel |
| Hysteresis | Dead band (°) to prevent oscillation near center |
| Left/Right sensor height | Sensor height above ground (mm) |
| Left/Right sensor width | Sensor distance from vehicle centerline (mm) |
| Left/Right sensor angle | Mount angle offset correction (°) |
| Effective spacing | Horizontal spacing used in lean calculation (mm) |
| Left/Right offset | Distance reading correction offset (mm) |
| Gyro axis | Which BMI160 axis is the yaw axis (0=X, 1=Y, 2=Z) |
| Invert gyro | Flip gyro sign if lean direction is reversed |
| Pulses per rev | Wheel speed sensor pulses per wheel revolution |
| Wheel circumference | Wheel circumference in meters |


## Build Notes

### Arduino IDE setup
- Board: `Seeed Xiao ESP32-C6` (from esp32 by Espressif package)
- Upload speed: 921600
- No external libraries required beyond the standard ESP32 Arduino core

### TMC2209 VREF setup
Set VREF trim pot to **~0.28V** measured on the pot wiper with a multimeter,
with motor disconnected and power applied. This sets motor current to ~0.2A.

### First boot checklist
1. Flash firmware via USB
2. With motor disconnected, power on and connect to AP
3. Go to `/test` and verify limit switches trigger correctly
4. Connect motor, power cycle, watch homing sequence on serial monitor (115200 baud)
5. Use `/test` to manually command projector angles and verify rotation direction
6. If rotation is reversed: swap either A1↔A2 or B1↔B2 motor coil wires
7. Calibrate sensor geometry and lean angle mapping via `/calibrate`

### Speed sensor wiring
The speed sensor input (D8/IO18) is currently stubbed out in firmware.
To enable:
1. Wire hall effect sensor signal wire to D8 with 10kΩ pull-up to 3.3V
2. In `setup()`, uncomment the two speed sensor lines:
   ```cpp
   pinMode(SPEED_PIN, INPUT_PULLUP);
   attachInterrupt(digitalPinToInterrupt(SPEED_PIN), speedPulseISR, RISING);
   ```
3. Configure pulses per revolution and wheel circumference on `/calibrate`

### Gyro axis calibration
On first install, use the IMU axis setting to identify which axis corresponds
to the bike's yaw (rotation around the vertical axis). With the bike level:
- Watch the raw gyro output on the dashboard while rotating the bike left/right
- The axis that changes is the yaw axis (usually Z=2 for flat-mounted IMU)
- Set "Invert gyro" if lean left shows as positive when it should be negative

## Known Issues / TODO

- Speed sensor not yet enabled in firmware (stubbed, see above)
- Lean angle fusion weighting (30/70 dist/IMU) is fixed — future: make configurable
- No thermal management (no heatsink sensor or fan in V3 hardware)
- Projector position is step-counted only — no encoder feedback

## Version History

| Version | Date | Notes |
|---|---|---|
| V1 | 2025 | LED bar, distance sensors only, Xiao ESP32-C6 |
| V2 | 2026-03 | Added BMI160 IMU, BuckTitan LED driver, MCP23008, INA219, DS18B20 |
| V3 | 2026-03 | Replaced LED bar with worm-drive stepper + bi-LED projector rotation |

