# JBOT Robot Project - Comprehensive Analysis

## System Overview

Two-motor hoverboard-style robot with:
- **Body**: Battery, STM32F103RCT6 main board, Pico 2 W, BNO055 IMU
- **STM32F103RCT6**: Handles motor control via BLDC controller
- **Pico 2W**: Handles balance stabilization at 200 Hz
- **Communication**: EFeru protocol over UART between STM32 and Pico

---

## 1. STM32F103RCT6 Subsystem

### Fixed Parameters (`config.h`)

| Parameter | Value (DEFAULT) | Value (TRANSPOTTER) | Description |
|-----------|-----------------|---------------------|-------------|
| `DEFAULT_FILTER` | 6553 (0.1f × 2¹⁴) | - | Filter coefficient, uint16_t with 2¹⁴ scaling |
| `DEFAULT_SPEED_COEFFICIENT` | 16384 (1.0f × 2¹⁴) | 14746 (0.9f × 2¹⁴) | Speed control coefficient |
| `DEFAULT_STEER_COEFFICIENT` | 8192 (0.5f × 2¹⁴) | 8192 (0.5f × 2¹⁴) | Steering control coefficient |
| `VARIANT_TRANSPOTTER` | - | FLASH_WRITE_KEY=0x1009, ROT_P=1.2, INVERT_R/L_DIRECTION | Transporter variant flags |

### PPM Control Validation Checks
- `PPM_MIN_CHECK`: 500 µs
- `PPM_MAX_CHECK`: 2500 µs
- `PPM_NEUTRAL_CHECK`: 1500 µs

### BLDC Motor Controller Interface (`BLDC_controller.h`)

**ExtU (Input Structures):**
- `motor_enable`: Boolean enable signal
- `control_mode_request`: Control mode selection
- `target_input`: Target value (speed/torque)
- `hall_sensor_states[3]`: Hall sensor A/B/C states
- `phase_currents[2]`: Phase currents (AB/BC)
- `dc_link_current`: DC link current measurement
- `mechanical_angle`: Motor mechanical angle

**ExtY (Output Structures):**
- `dc_duties[3]`: DC duty cycles for phases A/B/C

### EFeru Communication Protocol (`comms.h`, `comms.c`)

**Pico → STM32 Command Frame (8 bytes):**
```
[start u16][steer i16][speed i16][checksum u16]
```

**STM32 → Pico Feedback Frame (18 bytes):**
```
[start u16][cmd1 u8][cmd2 u8][speedR i16][speedL i16][batVoltage u16][boardTemp u16][cmdLed u8][checksum u16]
```

**Torque Mode (`TRQ_MODE`):**
- Values `+/-1000` map to `+/-I_MOT_MAX` (maximum motor current)

---

## 2. Pico 2W Subsystem

### Balance Control Algorithm (`control.rs`)

**Cascade PID Structure (200 Hz loop):**
```
Position Loop → Speed Loop → Acceleration Loop → Pitch Demand
                      ↓
                Inner Pitch PD (common-mode torque)
                      ↓
                Yaw Loop (differential torque)
```

**Tuning Parameters (`state.rs`):**
- `PITCH_MARGIN`: 4.0 (proportional pitch gain as multiple of A1/A2)
- `PITCH_DAMPING`: 1.6 (closed-loop pitch damping ratio)
- `CONTROL_HZ`: 200 (control loop rate)
- `MAX_PSCALE`: Maximum runtime gain scale factor

**Stop Reasons:**
- Latched off by fall detector, e-stop, or calibration invalidation

### Plant Model (`plant.rs`)

**Linearized Inverted Pendulum Dynamics:**
```
pitch_dd = A1*(pitch - slope) - A2*torque + S1*slope
x_dd     = B1*torque - B2*(pitch - slope) - S2*slope
```

Where:
- `pitch_dd`: Pitch acceleration
- `x_dd`: Linear acceleration
- `torque`: Applied torque
- `slope`: Ground inclination estimate
- `A1, A2, B1, B2, S1, S2`: Identified plant coefficients

### Pitch Reference Generator (`reference.rs`)

- **Algorithm**: Acceleration-limited trajectory
- **Default v_max**: 10 deg/s (0.1745 rad/s)
- Smoothly ramps pitch reference to target

### Signal Conditioning Filters (`filters.rs`)

- **LowPass**: First-order low-pass filter
- **SlewLimiter**: Per-wheel rate limit for torque commands

### BNO055 IMU Driver (`bno055.rs`)

**Configuration:**
- `IMUPLUS` mode: Accelerometer + gyroscope fused at 100 Hz
- Pico integrates gyro at 200 Hz (higher rate than IMU output)

**I2C Addresses:**
- BNO055 primary I2C address: 0x28 (configurable)

**Output Data:**
- Euler angles: pitch, roll, heading (degrees)
- Angular rates: gyro_x, gyro_y, gyro_z (deg/s)
- Acceleration: accel_x, accel_y, accel_z (m/s²)

**Mount Orientation Compensation (`mount.rs`):**
- Sensor mounted on its side (Z axis horizontal)
- Remaps sensor frame to robot frame (X forward, Y left, Z up)
- Uses vector-based remapping (exact, not Euler-angle approximations)

### Non-Volatile Settings (`nvm.rs`)

- **Record size**: 32 bytes (within 4KB flash erase block)
- **Magic**: `0x564E_424A` ("JBNV" in ASCII)
- **Stores**: Mount orientation, zero offsets, level reference
- **Validation**: CRC-check prevents silent corruption

### Spin Test Mode (`spin.rs`)

**Purpose**: Bring-up aid for wiring and sign checks
**Guards:**
- Torque clamped to `SPIN_TORQUE_LIMIT` (300 command units)
- Slew rate limited to `SPIN_SLEW_PER_S` (600 command units/s)
- Timeout after `SPIN_TIMEOUT_MS` (5000 ms)

---

## 3. Telemetry Structure (`state.rs`)

The `Telemetry` struct provides a comprehensive snapshot of robot state:

| Field | Type | Description |
|-------|------|-------------|
| `control_steps` | u32 | Control ticks since boot |
| `loop_hz` | u32 | Measured control loop rate |
| `imu_fresh` | bool | IMU sample freshness flag |
| `imu_calibrated` | bool | BNO055 calibration complete |
| `pitch_deg` | f32 | Measured pitch (control law input) |
| `roll_deg` | f32 | Measured roll |
| `heading_deg` | f32 | Measured heading |
| `gyro_*_dps` | f32 | Angular rates (deg/s) |
| `speed_left/right_rpm` | i16 | Wheel speeds |
| `battery_v` | f32 | Pack voltage |
| `board_temp_c` | f32 | Board temperature |
| `estop` | bool | Emergency stop latch |
| `speed_setpoint_mps` | f32 | Forward/backward speed setpoint |
| `steer_setpoint_rps` | f32 | Steering/yaw rate setpoint |
| `armed` | bool | Balance law arm latch |
| `balance_mode` | u8 | 0=disarmed, 1=transit, 2=hold, 3=recover, 4=fall |

---

## 4. Motor-STM32 Interaction

1. **Pico sends commands** via EFeru protocol (steer + speed setpoints)
2. **STM32 receives frames** through UART, validates checksum
3. **STM32 controls BLDC motors** using:
   - PID-like control with configurable coefficients
   - Hall sensor feedback for commutation
   - Phase current monitoring
   - DC link current protection
4. **STM32 sends feedback** to Pico with:
   - Wheel speeds (left/right RPM)
   - Battery voltage
   - Board temperature
   - Command echo for verification

---

## 5. Balance Calculation Methodology

### Step-by-Step:

1. **IMU Reading**: BNO055 provides fused accelerometer+gyro data at 100 Hz
2. **Mount Remapping**: Sensor frame → robot frame (accounts for vertical mounting)
3. **Zero Offset Application**: Subtract captured zero pitch/roll offsets
4. **Slope Trim**: Apply learned ground inclination compensation
5. **Pitch Demand Calculation** (cascade PID):
   - **Position loop**: Error = desired_position - actual_position → speed demand
   - **Speed loop**: Error = speed_demand - actual_speed → acceleration demand
   - **Acceleration loop**: Error = accel_demand - actual_accel → pitch demand
6. **Inner Pitch PD**: Common-mode torque for body stabilization
7. **Yaw Loop**: Differential torque for steering
8. **Torque Mixing**: Combine common-mode and differential torques
9. **Send to STM32**: EFeru command frame with steer/speed setpoints

### Key Design Points:
- **PITCH_MARGIN = 4.0**: Enough gain for stiff stance without exceeding motor torque capacity
- **PITCH_DAMPING = 1.6**: Over-damped for stability with mainboard transport delay
- **200 Hz loop rate**: Fast enough for balance, slow enough for computation
- **Runtime scaling (`pscale`)**: Allows fine-tuning gains without recompiling

---

## 6. Available Commands

### Via EFeru Protocol (Pico → STM32):
| Command | Description |
|---------|-------------|
| Speed setpoint | Forward/backward velocity (m/s) |
| Steer setpoint | Yaw rate (rad/s) |
| Spin test request | Direct wheel torque (bring-up aid) |
| Telemetry rate | Stream frequency (1-200 Hz) |
| Pitch trim | Zero offset adjustment |
| Home target | Explicit pitch reference |
| Reference rate | Max trajectory speed |
| Gain scale (`pscale`) | Runtime gain multiplier |

### Balance Modes:
| Mode | Value | Description |
|------|-------|-------------|
| Disarmed | 0 | No torque produced |
| Transit | 1 | Ramping to home reference |
| Hold | 2 | Parked on home reference |
| Recover | 3 | |pitch| above recover threshold |
| Fall | 4 | |pitch| passed fail threshold; law latched off |

---

## 7. Audit corrections (2026-11-22)

A fact-checked audit of the same code base is recorded in
`docs/audit-2026-11-22.md`. It supersedes the following optimistic statements in
this summary:

* The 200 Hz law is compiled but has never demonstrated a stable hold. Every
  armed run in the logs used `pscale 0.10-0.20`, below the 0.25 stability floor;
  the latest `arm` at boot-default `pscale=1.0` latched `pose_error` within 397 ms.
* `speed` and yaw were accepted by the console but not consumed by the committed
  control law; the working tree wires them in but has not been flashed. The new
  runtime `slew` knob is likewise unflashed and absent from `stat`.
* The mainboard serial timeout is ~800 ms in source (`SERIAL_TIMEOUT=160`), not
  the 100 ms quoted in `docs/pico-bridge.md`.
* `STEER_COEFFICIENT` is 0.5 in the `VARIANT_USART` build, while the Pico mixer
  inverse assumes 1.0; yaw differential will be half the requested value once
  yaw is enabled.
* `pscale` is a bench multiplier, not auto-tuning. No automatic gain search
  exists; only stillness-gated zero/level capture is automated.
