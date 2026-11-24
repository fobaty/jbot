//! Constants derived from the pinned EFeru build in `firmware/stm32f103/eferu`.
//!
//! These are hardware and contract facts, not tunable defaults. Changing them
//! requires a new measurement and an ADR, not a local edit.

/// HSI/2 (4 MHz) x PLL16, as flashed in `Src/main.c`.
pub const SYSTEM_HZ: u32 = 64_000_000;
pub const PWM_FREQ_HZ: u32 = 16_000;
/// 64 MHz / 2 / 16 kHz, center-aligned.
pub const PWM_ARR: u16 = 2_000;
pub const PWM_DEAD_TIME: u8 = 48;
pub const FOC_PERIOD_US: u32 = 62;
pub const MAIN_LOOP_MS: u32 = 5;

pub const A2BIT_CONV: i32 = 50;
pub const I_MOT_MAX_A: i32 = 10;
pub const I_DC_MAX_A: i32 = 12;
pub const N_MOT_MAX_RPM: i32 = 400;
pub const COMMAND_LIMIT: i16 = 1_000;

/// Number of ADC samples used to learn the zero-current offsets in the pinned C
/// build. Currents are untrusted until this many samples have arrived.
pub const ADC_OFFSET_SAMPLES: u32 = 2_000;
/// PCLK2/4 gives a 16 MHz ADC clock; 7.5 + 12.5 cycles is one conversion.
pub const ADC_CONV_CYCLES: u16 = 20;
/// TIM8 counter preload that aligns the ADC sample with the low-FET window.
pub const ADC_TRIGGER_OFFSET: u16 = 4 * ADC_CONV_CYCLES;
/// Duty margin kept by the FOC build so current sampling has a stable window.
pub const PWM_MARGIN: u16 = 110;
/// Full-scale q-axis current in fixdt(1,16,4): I_MOT_MAX * A2BIT_CONV * 16.
pub const I_MAX_Q4: i32 = I_MOT_MAX_A * A2BIT_CONV * 16;
/// DC-link chop threshold in raw ADC counts, from the pinned C build.
pub const DC_TRIP_COUNTS: i32 = I_DC_MAX_A * A2BIT_CONV;

/// Input filter alpha, fixdt(0,16,16). 19661 is the flashed alpha ~0.3.
pub const FILTER_ALPHA_Q16: u16 = 19_661;
/// Rate limit per 5 ms tick, fixdt(1,16,4). 480 >> 4 = 30 units/tick.
pub const RATE_Q4: i16 = 480;
/// Mixer coefficients, fixdt(0,16,14). The robot uses yaw coefficient 1.0;
/// the flashed reference left it at 0.5, which halves differential torque.
pub const SPEED_COEFF_Q14: i16 = 16_384;
pub const STEER_COEFF_Q14: i16 = 16_384;

pub const LEGACY_START_FRAME: u16 = 0xABCD;
pub const PROTOCOL_VERSION: u8 = 2;
pub const WIRE_MAX: usize = 256;
pub const COMMAND_TIMEOUT_US: u32 = 100_000;

/// PI gains are placeholders until they are re-derived from the generated C.
/// They must not be treated as a tuned controller.
pub const CURRENT_KP_Q14: i32 = 0;
pub const CURRENT_KI_Q14: i32 = 0;

// --- Battery monitoring, pinned config.h:75-88 ---
pub const BAT_FILT_COEF: u16 = 655;
pub const BAT_CALIB_REAL_VOLTAGE: i32 = 3_970;
pub const BAT_CALIB_ADC: i32 = 1_492;
pub const BAT_CELLS: i32 = 10;
pub const BAT_LVL5_ADC: i32 = (390 * BAT_CELLS * BAT_CALIB_ADC) / BAT_CALIB_REAL_VOLTAGE;
pub const BAT_LVL4_ADC: i32 = (380 * BAT_CELLS * BAT_CALIB_ADC) / BAT_CALIB_REAL_VOLTAGE;
pub const BAT_LVL3_ADC: i32 = (370 * BAT_CELLS * BAT_CALIB_ADC) / BAT_CALIB_REAL_VOLTAGE;
pub const BAT_LVL2_ADC: i32 = (360 * BAT_CELLS * BAT_CALIB_ADC) / BAT_CALIB_REAL_VOLTAGE;
pub const BAT_LVL1_ADC: i32 = (350 * BAT_CELLS * BAT_CALIB_ADC) / BAT_CALIB_REAL_VOLTAGE;
pub const BAT_DEAD_ADC: i32 = (337 * BAT_CELLS * BAT_CALIB_ADC) / BAT_CALIB_REAL_VOLTAGE;
pub const BAT_LVL1_ENABLE: bool = true;
pub const BAT_LVL2_ENABLE: bool = false;
pub const BAT_DEAD_ENABLE: bool = true;

// --- Board temperature, pinned config.h:102-110 ---
pub const TEMP_FILT_COEF: u16 = 655;
pub const TEMP_CAL_LOW_ADC: i32 = 1_655;
pub const TEMP_CAL_LOW_DECIC: i32 = 358;
pub const TEMP_CAL_HIGH_ADC: i32 = 1_588;
pub const TEMP_CAL_HIGH_DECIC: i32 = 489;
pub const TEMP_WARNING_DECIC: i32 = 600;
pub const TEMP_POWEROFF_DECIC: i32 = 650;
// The reference ships these disabled because its NTC calibration is unverified.
pub const TEMP_WARNING_ENABLE: bool = false;
pub const TEMP_POWEROFF_ENABLE: bool = false;
pub const STANDSTILL_RPM: i32 = 20;

// --- Motor diagnostics, generated BLDC_controller_data.c:189-197,266-274 ---
pub const DIAG_ENABLE: bool = true;
pub const DIAG_QUALIFY_TICKS: u32 = 1_280;
pub const DIAG_DEQUALIFY_TICKS: u32 = 9_600;
pub const DIAG_STILL_SPEED: i32 = 48;
pub const DIAG_INPUT_TARGET_THRESHOLD: i32 = 9_600;
pub const HALL_POSITION_LUT: [u8; 8] = [0, 2, 0, 1, 4, 3, 5, 0];

// --- Input qualification and inactivity, pinned config.h:186-190 ---
pub const ADC_PROTECT_THRESH: i32 = 200;
pub const ADC_PROTECT_TIMEOUT: u32 = 100;
pub const INACTIVITY_TICKS: u32 = 80_000;
pub const INACTIVITY_CMD_THRESHOLD: i16 = 50;

// --- Buzzer, pinned bldc.c:150-167 (f ≈ 8000 / freq Hz) ---
pub const BUZZER_BURST_TICKS: u32 = 5_000;
pub const BUZZER_LOW_FREQ: u16 = 24;
pub const BUZZER_MED_FREQ: u16 = 10;
pub const BUZZER_HIGH_FREQ: u16 = 5;
pub const FOC_TICKS_PER_MS: u32 = 16;
