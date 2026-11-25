# Rust torque node vs. pinned EFeru C firmware — full comparison

Scope: the flashed C build is `VARIANT_USART` with `BOARD_VARIANT 0`
(`firmware/stm32f103/eferu-foc/eferu/Inc/config.h:66`, `:327-362`, patched by
`eferu-jbot.patch`). The Rust build is
`firmware/stm32f103/rust/f103_core` + `f103_fw` in its current
**full-behaviour** configuration. Lines below cite the source of each fact.

## 1. Timing and hardware constants

| Item | EFeru C | Rust | Match |
|---|---|---|---|
| MCU clock | HSI/2 x PLL16 = 64 MHz (`setup.c:175`) | `SYSTEM_HZ = 64_000_000` (`config.rs:7`) | yes |
| PWM | 16 kHz centre-aligned, ARR 2000 (`config.h:28`, `setup.c:498-505`) | `PWM_FREQ_HZ 16000`, `PWM_ARR 2000` (`config.rs:8-10`) | yes |
| Dead time | 48 (`config.h:29`, `setup.c:522-529`) | `PWM_DEAD_TIME 48` (`config.rs:11`) | yes |
| FOC tick | 16 kHz DMA TC IRQ (`bldc.c:104`) | DMA1_CH1 ISR, `FOC_PERIOD_US 62` (`config.rs:12`, `main.rs:212`) | yes |
| Main loop | 5 ms (`config.h:33`, `main.c:263`) | `MAIN_LOOP_MS 5` (`config.rs:13`) | yes |
| Current scale | `A2BIT_CONV 50` (`config.h:36`) | same (`config.rs:15`) | yes |
| Motor current limit | `I_MOT_MAX 10 A` → 8000 Q4 (`config.h:162-164`, `util.c:238`) | `I_MOT_MAX_A 10`, `I_MAX_Q4 8000` (`config.rs:16,31`) | yes |
| DC chop threshold | `I_DC_MAX 12 A` → 600 counts (`config.h:165`, `bldc.c:52`) | `I_DC_MAX_A 12`, `DC_TRIP_COUNTS 600` (`config.rs:17,33`) | yes |
| Speed limit | `N_MOT_MAX 400 rpm` → 6400 Q4 (`config.h:166`, `util.c:239`) | `N_MOT_MAX_RPM 400` (`config.rs:18`) | yes |
| Control type | `FOC_CTRL = 2` (`config.h:157`) | `z_ctrlModReq = 3` (TRQ) in `main.rs:159-172` | yes |
| Control mode | `TRQ_MODE = 3` (patched, `config.h:158`) | TRQ mode (`main.rs:160,174`) | yes |
| Input filter | `FILTER 19661` = 0.3 (`config.h:353-356`, `main.c:328-334`) | `FILTER_ALPHA_Q16 19661` (`config.rs:36`) | yes |
| Rate limit | `RATE 480` = 30/tick (`config.h:198-199`, `main.c:329-330`) | `RATE_Q4 480` (`config.rs:38`) | yes |
| Mixer speed coeff | `16384` = 1.0 (`config.h:201`) | `SPEED_COEFF_Q14 16384` (`config.rs:41`) | yes |
| Mixer steer coeff | `8192` = 0.5 (`config.h:202`) | `STEER_COEFF_Q14 8192` (`config.rs:42`) | yes |
| Command timeout | `SERIAL_TIMEOUT 160` ≈ 800 ms (`config.h:670`, `util.c:147-150`) | `COMMAND_TIMEOUT_US 800_000` (`config.rs:47`) | yes |
| Command range | ±1000 (`util.c:266-274`) | `COMMAND_LIMIT 1000` (`config.rs:19`) | yes |
| Startup latch delay | `STARTUP_DELAY_MS 500` (`config.h:112-114`, `main.c:205`) | not implemented — latch driven immediately (`hal.rs:251`) | **gap** |
| ADC offset samples | 2000 (`util.c:...`) | `ADC_OFFSET_SAMPLES 2000` (`config.rs:23`) | yes |
| PWM sampling margin | `pwm_margin 110` for FOC (`bldc.c:169-174`) | `PWM_MARGIN 110` (`config.rs:29`) | yes |

## 2. GPIO map

| Function | Pin | C source | Rust source |
|---|---|---|---|
| Status LED | PB2, push-pull, steady high after boot | `defines.h:117-118`, `setup.c:407-410`, `main.c:221-222` | `hal.rs:250-252` (PB2 set) |
| Buzzer | PA4, push-pull, toggled in 16 kHz ISR | `defines.h:120-122`, `bldc.c:150-166` | `hal.rs:249`, `set_buzzer()` `hal.rs:628` |
| Power latch (OFF) | PA5, active high | `defines.h:132-134`, `main.c:214`, `util.c:1573` | `hal.rs:251`, `release_power_latch()` `hal.rs:...` |
| Power button | PA1, active high, **no pull** | `defines.h:140-142`, `setup.c:401-404`, `util.c:1580` | PA1 with **pull-down** (`hal.rs:...`) to avoid self-poweroff on a bare board |
| Charge detect | PA12, pull-up, **unused** in C | `defines.h:148-150`, `setup.c:389-391` | not configured |
| USART2 TX/RX | PA2 / PA3, 115200 8N1 | `config.h:672-676`, `setup.c:124-136` | `hal.rs:236-237` |
| USART3 TX/RX | PB10 / PB11, 115200 8N1 | `config.h:678-683`, `setup.c:179-191` | `hal.rs:238-239` |
| Left halls | PB5/PB6/PB7, active low | `bldc.c:193-195` | `main.rs:125-129` (inverted) |
| Right halls | PC10/PC11/PC12, active low | `bldc.c:239-241` | `main.rs:130-134` (inverted) |
| Left PWM U/V/W | PC6/PA7, PC7/PB0, PC8/PB1 (TIM8) | `defines.h:45-60` | `hal.rs:212-217` |
| Right PWM U/V/W | PA8/PB13, PA9/PB14, PA10/PB15 (TIM1) | `defines.h:62-77` | `hal.rs:219-225` |
| ADC currents | RL-DC PC0, RL-A PA0, RL-B PC3, RR-DC PC1, RR-B PC4, RR-C PC5 | `defines.h:83-101` | `hal.rs:241-245` |
| ADC pack / temp | PC2 / internal sensor | `defines.h:106-112`, `setup.c:641-645` | `config.rs:57`, `hal.rs:241-245` |
| ADC DMA order | `dcr,dcl,rlA,rlB,rrB,rrC,batt1,l_tx2,temp,l_rx2` | `defines.h:219-230` | `sensing.rs:15-26` |
| CAN | not configured; HAL CAN disabled | `stm32f1xx_hal_conf.h:54` | not implemented |

## 3. UART roles and wire contracts

### C build
- **USART2** (PA2/PA3): legacy command RX + legacy feedback TX, 115200 8N1
  (`config.h:328-330`, `util.c:1091-1126`, `main.c:521-546`).
- **USART3** (PB10/PB11): debug **text only**, 115200 8N1
  (`config.h:358-361`, `util.c:205-218`). The C RX path for `$…` commands is
  compiled out because `DEBUG_SERIAL_PROTOCOL` is undefined (`config.h:263`,
  `util.c:1159-1176`).
- **Command**, 8 bytes (`util.h:38-43`, `util.c:1295-1303`):
  `CD AB <steer i16 LE> <speed i16 LE> <xor16 LE>`.
- **Feedback**, 18 bytes / 9 u16 (`main.c:124-140, 521-546`):
  `CD AB, cmd1, cmd2, speedR, speedL, batVoltage, boardTemp, cmdLed, xor`.
- Debug text every ~125 ms:
  `in1:… in2:… cmdL:… cmdR:… BatADC:… BatV:… TempADC:… Temp:…` (`main.c:501-518`).

### Rust build
- **USART2**: same legacy 8-byte command and 18-byte feedback at 100 Hz
  (`main.rs:439-453`, `:542-555`; `protocol.rs:49-62`, `:123-156`).
- **USART3**: full bidirectional **console** (`main.rs:584-714`) plus a binary
  trace stream sharing the same TX line. Commands: `$HELP`, `$GET`, `$GETP`,
  `$WATCH`, `$SET WATCH`, `$SET PROTOCOL`, `$SET <KEY>`, `$TRACE`, `$ARM`,
  `$DISARM`, `$INIT`, `$SAVE`, `$DUMP`, `$ADC` (`console.rs:362-368`).
- **Hardened COBS/CRC16 frame** with version/seq/timestamp and a 25-byte
  feedback payload is compiled and golden-tested (`protocol.rs:123-194`,
  `:708-755`) but **refused** in this full-behaviour build — `$SET PROTOCOL
  HARDENED` is rejected (`main.rs:643-652`).
- **Trace frame**, msg type 3, 88-byte native-width payload of both motors'
  internal FOC state (`trace.rs:12-140`), streamed at ≤100 Hz because
  `$TRACE` clamps non-zero decimation to ≥160 (`main.rs:653-673`).
- Pico mirror lives in `firmware/pico2w/core/src/hardened.rs`.

## 4. User controls and indications

| Behaviour | C | Rust |
|---|---|---|
| Startup | 500 ms delay → latch → melody 8→0 (100 ms/tone) → LED on (`main.c:205-222`, `util.c:419-426`) | latch → LED on → same melody (`hal.rs:250-252`, `main.rs:...`, `alerts.rs:Melody::power_on`) |
| Motor enable | no fault and `|cmdL|<50, |cmdR|<50` → 2 short beeps (6,4), `enable=1` (`main.c:269-279`) | identical auto-arm plus `ArmBeep [6,0,4,0]` (`main.rs` link callback, `alerts.rs:ArmBeep`) |
| Button short press | `enable=0`, wait release, melody 0→7, `saveConfig()`, PA5 low, halt (`util.c:1578-1608`, `:1561-1575`) | same: disarm, 80 ms debounce, `Melody::power_off()`, `$SAVE` equivalent, PA5 low, watchdog-fed halt (`main.rs` loop) |
| Button long press (≥5 s) | ADC limit calibration `adcCalibLim()` (`util.c:1596-1601`) | **not implemented** |
| Button double press | current/speed limit calibration `updateCurSpdLim()` (`util.c:1591-1595`, `:612-661`) | **not implemented** |
| Board LED | steady high after boot (`main.c:222`) | same (PB2 set in `hal.rs`) |
| Sideboard LED bitmask in feedback | battery levels, motor error, timeouts, backward (`util.c:1351-1417`) | `led` field is always 0 (no sideboards in this build) |
| Battery alerts | LVL1 fast medium beeps, LVL2 slow, dead → poweroff (`config.h:79-81`, `main.c:571-575`) | `Alert::BatteryLevel1/2`, `BatteryDead` → continuous urgent tone or latch-off (`alerts.rs:135-146`, `main.rs`) |
| Temperature alerts | warning 600 (disabled), poweroff 650 (disabled) (`config.h:107-110`) | same thresholds and same disabled defaults (`config.rs:75-79`) |
| Motor diagnostic | 1 low beep, `enable=0` (`main.c:576-578`) | `Fault::MotorDiagnostic` + alert, safe output, no latch-off |
| Serial timeout | 3 low beeps, OPEN_MODE, zero torque, latch stays on (`main.c:581-582`, `util.c:1031-1038`) | `Alert::SerialTimeout` (3 bursts), `Freewheel`, latch stays on |
| Backward drive | fast high beep `beepCount(0,5,1)` (`main.c:591-593`) | `Alert::Backward` → high frequency (`alerts.rs:143`) |
| Inactivity | ~400 s → `poweroff()` (`main.c:600-619`) | `INACTIVITY_TICKS 80000` → `poweroff()` (`config.rs:93`, `main.rs`) |

## 5. Parameter availability and persistence

### C build
- **Runtime console parameter changes do not exist**: `comms.c` is compiled out,
  so `SAVE`/`SET` commands are unreachable (`comms.c:32-33`, `config.h:263`).
- Persistent flash record with `FLASH_WRITE_KEY = 0x1002`
  (`config.h:336-347`, `util.c:304-305`):
  slot 1 = `i_max`, slot 2 = `n_max`, slots 3–10 = input1/2 type/min/mid/max.
  Written only by `saveConfig()` during `poweroff()` (`util.c:1535-1555, 1572`).
- `updateCurSpdLim()` (double-press) scales `i_max` and `n_max` from the input
  range and stores them (`util.c:612-661`).

### Rust build
- **13 runtime tunables**, readable with `$GETP` and changed with
  `$SET <KEY> <VALUE>` (`parameters.rs:178-192`, `console.rs:362-368`):

  | Key | Meaning | Default | Hard clamp |
  |---|---|---|---|
  | `IMAX` | Q4 current limit | 8000 | ≤ 8000 |
  | `NMAX` | Q4 speed limit | 6400 | ≤ 6400 |
  | `STEER` | yaw mixer coeff Q14 | 8192 | ≤ 16383 |
  | `SPEED` | speed mixer coeff Q14 | 16384 | ≤ 16383 |
  | `IDKP`, `IDKI` | id PI gains | 819 / 737 | ≤ 65535 |
  | `IQKP`, `IQKI` | iq PI gains | 1229 / 1229 | ≤ 65535 |
  | `NKP`, `NKI` | speed PI gains | 4833 / 251 | ≤ 65535 |
  | `CURRFILT` | current filter | 7864 | ≤ 65535 |
  | `VDMAX` | d-axis voltage cap | 14400 | ≤ 14400 |
  | `OPENRATE` | open-loop ramp | 12288 | ≤ 32767 |

- Actuator keys are refused while armed; mixer keys are not
  (`parameters.rs:269-276`, `main.rs:607-625`).
- `$SAVE` writes a 16-word keyed, versioned, CRC-checked record
  (`FLASH_WRITE_KEY 0x1002`, version 2, CRC16, `parameters.rs:14-16, 99-113`),
  and is refused while armed because flash erase stalls instruction fetch
  (`main.rs:680-698`).
- `$GET` exposes tick, overcurrent/skip counters, calibration state, protocol,
  safe mode, arm/fault/faults/rejected, link age, steer/speed, six duties,
  battery, temperature, diagnostics, DWT `cyc/max/miss` and NVM source
  (`console.rs:370-421`).
- `$DUMP` is a 32-sample blackbox (`console.rs:447-...`); `$ADC` prints the ten
  packed ADC fields (`console.rs:437-444`, `sensing.rs:15-26`).

**Gap:** the C build's input calibration (`min/mid/max`) and the
`updateCurSpdLim()` scaling are not exposed in the Rust console. The Rust NVM
stores the 13 tunables instead of the C firmware's input-calibration slots.
`FILTER` and `RATE` are pinned constants, not tunables.

## 6. FOC controller and observability

- The Rust port is bit-exact: 120/120 ticks for both golden CSVs, all eight
  outputs asserted (`foc_generated.rs:1449-...`, `tools/foc_golden/`).
- The controller gets the same `b_motEna`, `z_ctrlModReq`, `r_inpTgt`, hall and
  current inputs (`main.rs:158-185`; `bldc.c:188-252`).
- C exposes only `n_mot`, `iq`, `id`, angle and `z_errCode` in feedback/RAM.
  Rust additionally streams the full internal state (scheduler token, both PI
  integrators at full 32-bit width, Vd/Vq, phase amplitudes, DC currents and
  six duties) in the 88-byte trace (`trace.rs`, `f103_trace.py`).
- Safe-state tests and golden vectors: `f103_core` 73/73 passing,
  `pico_core` 142/142.

## 7. Summary of differences

1. **MOE is never driven by the Rust firmware.** Even with auto-arming, no gate
   can switch until HIL validation and Safety-Reviewer sign-off. The C build
   sets MOE every FOC tick when `enable=1` and DC current is within budget
   (`bldc.c:136-148`).
2. **Rust adds a console and a binary trace** where C had only a one-way debug
   text line; this is the main calibration advantage.
3. **Rust adds 13 clamped runtime tunables and explicit `$SAVE`** where C had
   none (and only two persisted limits).
4. **Rust omits long-press ADC calibration and double-press limit calibration**,
   and has no 500 ms startup latch delay.
5. **Rust keeps both wire contracts compiled but refuses hardened** in this
   full-behaviour build, matching the C legacy-only link.
6. **Button PA1 has a pull-down** in Rust instead of the C no-pull input, to
   prevent a floating input from self-powering-off a bare board.
7. **No sideboards**: the feedback `led` word is always 0.
8. **CAN is absent in both builds.**
