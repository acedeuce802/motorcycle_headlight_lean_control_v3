# Motorcycle Adaptive Cornering Lights V3

Lean-angle-controlled bi-LED projector rotation system for motorcycles.
Uses a worm-drive stepper motor to physically rotate a bi-LED projector
in response to lean angle, keeping the beam pointed into corners.

## Hardware

| Component | Part | Notes |
|---|---|---|
| Microcontroller | Seeed Xiao ESP32-C6 | |
| IMU | BMI160 breakout | Wired directly to D4/D5, addr 0x68 |
| I2C mux | TCA9548A breakout | addr 0x70 |
| Distance sensors | DFRobot SEN0590 ×2 | TCA channels 0 (left) and 1 (right) |
| Stepper driver | TMC2209 module | Standalone mode, 1/8 microstepping |
| Stepper motor | NEMA 8, 1.8°/step | 8HS11-0204S, 0.2A rated |
| Worm drive | M1.5, 72:1 ratio | Brass worm shaft, PET-CF printed wheel |
| Limit switches | Micro switch ×2 | Active LOW, left and right travel stops |
| Position pot | 10kΩ single-turn | Concentric mount on projector shaft |
| Bearings | 6816-2RS (front), 6704ZZ (worm shaft) | Projector rotation |
| Regulator | MP1584 module | 12V → 5V for ESP32 and TMC VIO |


## Pin Assignments (Xiao ESP32-C6)

| Pin | GPIO | Signal | Notes |
|---|---|---|---|
| D0 | IO0 | STEP | TMC2209 step pulse |
| D1 | IO1 | DIR | TMC2209 direction |
| D2 | IO2 | EN | TMC2209 enable (active LOW) |
| D3 | IO21 | TMC UART TX | Via 1kΩ to PDN_UART pad |
| D4 | IO22 | I2C SDA | TCA9548A + BMI160 |
| D5 | IO23 | I2C SCL | TCA9548A + BMI160 |
| D6 | IO16 | LIMIT LEFT | Micro switch, INPUT_PULLUP, active LOW |
| D7 | IO17 | LIMIT RIGHT | Micro switch, INPUT_PULLUP, active LOW |
| D8 | IO19 | SPEED | Hall effect pulse input (stubbed) |
| D9 | IO20 | TMC UART RX | Direct to PDN_UART pad (same node as TX) |
| D10 | IO18 | POT ADC | Position pot wiper, ADC input |
| 5V | — | Power in | From MP1584 (12V → 5V) |
| 3V3 | — | Power out | To TCA9548A VCC, BMI160 VIN |
| GND | — | Ground | Common ground |

**Note on GPIO numbering:** The Xiao ESP32-C6 D-pin labels do NOT map 1:1 to GPIO
numbers. Always use GPIO numbers in firmware (D0=IO0, D1=IO1, D2=IO2, D3=IO21,
D4=IO22, D5=IO23, D6=IO16, D7=IO17, D8=IO19, D9=IO20, D10=IO18).

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
| Motor | NEMA 8, 1.8°/step (200 steps/rev), rated 0.2A |
| Microstepping | 1/8 (MS1=GND, MS2=GND) |
| Worm ratio | 72:1 |
| Steps per degree | 320 steps/° |
| Full travel | ±45° (set by limit switches at homing) |
| Homing speed | 800µs/step |
| Normal speed | 200µs/step |
| Motor current | 300mA via UART (validated: ~60-65°C surface at 100% duty cycle) |

## TMC2209 Current Control (UART)

Current is set via UART on every boot — the VREF trim pot is bypassed.
This requires a specific wiring setup and no TMCStepper library (raw protocol):

**Wiring:**
```
ESP32 D3/IO21 ── 1kΩ ──┬── TMC TX pad (PDN_UART)
ESP32 D9/IO20 ──────────┘
10kΩ pull-up from the same node to 3.3V
```

The TX and RX pads on the TMC2209 module are shorted together (confirmed with
multimeter continuity). Both pads connect to the same PDN_UART chip pin.
The 1kΩ resistor on the TX side prevents bus contention when the TMC responds.

**Why no TMCStepper library:** Including `<TMCStepper.h>` causes the library to
claim Serial1 at startup even with no objects constructed, corrupting IO0/IO1/IO2
(STEP/DIR/EN). Raw UART writes avoid this entirely.

**GCONF note:** Writing `0x00000000` to GCONF sets `I_scale_analog=0` (use digital
current register, not VREF pin). Do NOT set `pdn_disable=1` — this disables the
hardware STEP/DIR/EN interface. Do NOT set `mstep_reg_select=1` — this overrides
MS1/MS2 pins and defaults to 256 microsteps (motor goes very slow).

**Rsense:** Module sense resistors read `30E` on the silkscreen (non-standard
marking). Empirically estimated at ~0.17Ω based on TMCStepper readback of 183mA
at 0.25V VREF. The firmware uses R_SENSE=0.11 in the CS calculation — this
results in IRUN=2 at 200mA target, IRUN=3 at 300mA target.

**Motor current setting:** Configurable on the web calibration page. Applied on
every boot. Default: 300mA.
- 300mA validated: ~60-65°C motor surface temp at 100% continuous duty cycle
- Real-world duty cycle is much lower (motor disabled when |lean| < dead zone)
- Expected motor temp in real use: well under 50°C


## Position Potentiometer (Closed-Loop Feedback)

A 10kΩ single-turn pot is mounted concentric to the projector output shaft
(1:1 direct drive). Its wiper feeds into D10/IO18 (ADC, 12-bit, 0–4095 counts).

**Why closed loop:** The worm drive is self-locking, so the projector won't drift
when the motor is off. However, missed steps during fast moves or homing would
cause the step counter to drift from reality. The pot gives true position feedback
and eliminates this concern.

**ADC wiring — series resistors for linearity:**
The ESP32 ADC is nonlinear near both rails (0–0.1V and 3.2–3.3V). Limit the
pot sweep to the linear region by adding small resistors on the pot supply pins:

```
3.3V ── 1kΩ ── pot end 1 (CCW)
GND  ── 1kΩ ── pot end 2 (CW)
pot wiper ──── D10/IO18 (ADC)
100nF cap from wiper to GND (noise filter)
```

This limits the ADC range to roughly 0.3V–3.0V, staying within the linear region.
Without these resistors the pot at full travel reads incorrectly near the ends.

**Calibration — automatic during homing:**
1. Homing drives to left limit switch → records `potLeftADC`
2. Homing drives to right limit switch → records `potRightADC`
3. Center = `(potLeftADC + potRightADC) / 2 + potCenterOffset`
4. All values saved to NVS (persistent across power cycles)
5. `usePot` flag set to `true` in NVS

**After first homing, homing is skipped on subsequent boots:**
On startup, if `usePot=true` and `potLeftADC != potRightADC` in NVS, the system
reads the pot, calculates current angle, and goes directly to normal operation
without touching the limit switches.

**Fine-tuning center:** Use the `Center Offset (ADC counts)` field on the
calibration page to shift the zero position after homing. Saves to NVS without
needing a rehome. Rule of thumb: each 10 ADC counts ≈ 0.2° at full travel.

**Deadband:** `potDeadband` (default 30 ADC counts) prevents hunting at the
target position. Increase if the projector oscillates, decrease for tighter
tracking.


## Passive Components

### TMC2209 power rail (12V → VM)
- 10Ω 1W resistor in series on VM line (transient filter — NOT in main 12V path)
- 470µF 35V electrolytic from VM to GND
- 100nF ceramic from VM to GND
- P6KE16CA TVS diode: across VM/GND (shunt protection, not inline)

### TMC2209 UART
- 1kΩ resistor: D3/IO21 to TMC TX/RX pad
- 10kΩ pull-up: TMC TX/RX pad to 3.3V
- Remove any 100nF cap that was on RX pad to GND (present on some modules)

### TCA9548A I2C channels
- 4.7kΩ pull-up on each downstream SDA/SCL channel in use (SD0/SC0, SD1/SC1)
- 100nF ceramic from VCC to GND

### BMI160
- SA0 → GND (sets I2C address 0x68)
- CS → 3.3V (ensures I2C mode)

### Position potentiometer
- 1kΩ from 3.3V to pot CCW end
- 1kΩ from GND to pot CW end
- 100nF from wiper to GND
- Wiper → D10/IO18

### Limit switches
- INPUT_PULLUP enabled in firmware (internal ~45kΩ pull-up)
- Optional: 100nF ceramic from signal pin to GND for debounce

### Speed sensor input
- 10kΩ from D8/IO19 to 3.3V (if open-collector hall effect)
- 2.2kΩ from D8/IO19 to GND (voltage divider with 10kΩ if 12V signal)
- 100nF from signal pin to GND

## Homing Sequence

Homing is triggered manually via the **"Run Homing & Set Zero"** button on the
calibration page, or via `/api/setzero`. It does NOT run automatically on boot
once the pot has been calibrated.

1. **Left sweep** — motor rotates toward left limit switch at homing speed.
   Triggers when switch goes LOW. Records pot ADC as `potLeftADC`.
2. **Back-off** — retracts 100 steps away from left limit.
3. **Right sweep** — motor rotates toward right limit switch, counting steps.
   Triggers when switch goes LOW. Records pot ADC as `potRightADC`.
4. **Back-off** — retracts 100 steps from right limit.
5. **Center** — moves to midpoint. Sets `potCenterADC = (left+right)/2 + offset`.
   Saves all pot calibration to NVS. Sets `usePot=true`.

After first homing, subsequent boots skip homing entirely and use the pot to
determine current position. Limit switches remain active as crash failsafes.

If either limit switch is not found within 50,000 steps, homing fails and the
stepper is disabled. Fix limit switch wiring and press the button again.


## Control Strategy

**Dead zone:** When |lean| < `deadZone` (default ±5°), the target is forced to
center (0°) and the motor is disabled once it arrives. The worm gear holds the
projector position with zero current draw.

**Tracking:** When |lean| ≥ `deadZone`, the motor enables and steps toward the
target angle derived from the lean angle. In closed-loop mode the pot error drives
the direction. In open-loop the step counter drives it.

**Auto enable/disable:** The motor is only enabled when steps are needed. On
arrival at target (within deadband/1 step) it disables immediately. This keeps
thermal load low and relies on the worm gear's self-locking for position holding.

**Duty cycle:** At real riding duty cycles (motor off on straights, stepping only
during active cornering) the motor thermal load is minimal — far below the
100% continuous test case.

## Lean Angle Sources

### Distance sensors (geometry-based)
Both SEN0590 sensors measure distance to the ground. Difference in readings
divided by sensor spacing gives lean angle via `atan2`. Valid only when both
sensors return readings within the configured min/max distance window.

### BMI160 gyro (Reidel model)
```
lean = atan(speed_m_s × yaw_rate_rad_s / 9.81)
```
Active only when speed > 2 m/s. Gyro zero bias calibrated at startup.
IMU yaw axis confirmed as **X-axis** on this PCB layout.

### Fusion
When both sources are available and speed > 2 m/s:
```
lean_fused = 0.3 × lean_distance + 0.7 × lean_IMU
```

## Web Interface

Connect to WiFi AP (default SSID: `CL-V3-XXXXXX`, password: `cornering123`)
then browse to `http://192.168.5.1` or `http://CL-V3-XXXXXX.local`.

| Page | URL | Description |
|---|---|---|
| Dashboard | `/` | Live lean, projector angle, pot ADC, motor state |
| Calibration | `/calibrate` | Lean mapping, pot settings, motor current, sensor geometry |
| Config | `/config` | WiFi, device name, sensor source enable/disable |
| Test | `/test` | Manual projector angle control, test mode |
| OTA | `/update` | Web-based firmware upload |
| Log | `/log` | 5Hz datalog with CSV download |

### API endpoints

| Endpoint | Method | Description |
|---|---|---|
| `/api/status` | GET | JSON: all live state values including potADC |
| `/api/calibrate` | POST | Save calibration settings |
| `/api/config` | POST | Save config and restart |
| `/api/test` | GET/POST | Set/release test mode angle |
| `/api/setzero` | GET | Run full homing sequence, save pot calibration |
| `/api/rehome` | GET | Alias for setzero |
| `/api/reset` | GET | Factory reset (clears NVS, including pot cal) |
| `/api/logrow` | GET | Single JSON row for datalogger |


## Configuration (Calibration Page)

| Setting | Description |
|---|---|
| Dead zone | Lean below this (°) keeps projector at center, motor disabled |
| Max lean angle | Lean angle (°) that maps to full projector travel |
| Position hysteresis | Dead band (°) near target to prevent oscillation |
| Motor current (mA) | TMC2209 RMS current, applied via UART on next boot |
| Pot deadband | ADC counts of deadband around target (default 30) |
| Center offset | Fine-tune zero after homing (ADC counts, ±200) |
| Sensor geometry | Height, width, angle for each distance sensor |
| Gyro axis | Which BMI160 axis is yaw (X=0 confirmed on this layout) |
| Speed sensor | Pulses per rev, wheel circumference |

## Build Notes

### Arduino IDE setup
- Board: `Seeed Xiao ESP32-C6` (esp32 by Espressif package)
- Upload speed: 921600
- Libraries required: none beyond ESP32 Arduino core (TMCStepper NOT used)

### First boot checklist
1. Flash firmware via USB
2. Power on (12V + USB) — TMC UART current init runs first, then homing attempt
3. Watch serial monitor (115200 baud) for TMC UART output and homing progress
4. Connect to WiFi AP, go to `/calibrate`, verify motor current is set correctly
5. Wire pot to D10/IO18 with series resistors (see Passive Components)
6. Use `/test` to verify limit switches and motor direction
7. If rotation is reversed: swap either A1↔A2 or B1↔B2 motor coil wires
8. Press "Run Homing & Set Zero" button on calibration page
9. Verify pot ADC values are reasonable (left < center < right or right < center < left)
10. Fine-tune center with the Center Offset field if needed

### Motor coil wiring (from spec sheet)
| Wire color | Coil | TMC2209 pin |
|---|---|---|
| Black | A+ | A1 |
| Green | A− | A2 |
| Red | B+ | B1 |
| Blue | B− | B2 |

**Never disconnect motor wires while TMC2209 is powered** — inductive spike can
destroy the driver. Always power off first.

### Gyro axis confirmation
IMU X-axis is the yaw axis on this PCB layout (confirmed by bench test —
X-axis responds to left/right rotation of the PCB). Default `imuYawAxis = 0`.

### Speed sensor (currently stubbed)
To enable: uncomment the two speed sensor lines in `setup()` and configure
pulses per revolution and wheel circumference on the calibration page.

## Known Issues / TODO

- Speed sensor not yet enabled in firmware (stubbed)
- Lean angle fusion weighting (30/70) is fixed — future: make configurable
- TMC2209 UART current writes cannot be read back (echo draining issue with
  single-wire half-duplex) — write-only, verified by thermal test instead
- Pot closed-loop not yet field-tested (pot not yet wired at time of writing)

## Version History

| Version | Date | Notes |
|---|---|---|
| V1 | 2025 | LED bar, distance sensors only |
| V2 | 2026-03 | Added BMI160, BuckTitan driver, MCP23008, INA219, DS18B20 |
| V3 | 2026-03 | Worm-drive stepper + bi-LED projector rotation |
| V3.1 | 2026-04 | TMC2209 UART current control, pot closed-loop feedback, smart boot |
