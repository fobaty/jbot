//! JBOT real-time controller firmware for the Raspberry Pi Pico 2 W (RP2350).
//!
//! This is the foundation stage of the Tier 1 controller described in
//! `docs/pico-bridge.md`: it brings up the clock tree, the UART link to the
//! STM32F103 torque node, the I2C link to the BNO055, the 200 Hz control tick,
//! and a USB console for bench work.
//!
//! Tasks:
//!
//! * [`imu_task`] samples the BNO055 at 100 Hz and publishes the latest sample.
//! * [`link_rx_task`] parses EFeru feedback frames from the mainboard.
//! * [`control_task`] runs at 200 Hz, computes the balance command, sends an
//!   8-byte frame to the mainboard, toggles the GP10 marker, and feeds the
//!   hardware watchdog.
//! * [`console::usb_task`] and [`console::console_task`] serve the USB console.
//!
//! The balance law is the cascade from `jbot_pico_core::control`: position to
//! speed to acceleration to a pitch demand, an inner pitch PD that produces
//! the common-mode torque, and a yaw loop that may only use the remaining
//! torque headroom. Its gains are derived from the identified plant once per
//! boot, so there is no magic gain table in this file.
//!
//! Safety: if the IMU sample is stale, or the emergency stop is latched, the
//! task sends [`Command::ZERO`]. The mainboard independently stops on its own
//! serial timeout. Arming is refused when the torque link is blind, when the
//! pose is outside the lift budget of the torque available, or when the body
//! is already on the ground; while armed, the same limits plus a travel limit
//! latch the law off. The bench spin test applies a limited, slew-limited,
//! self-expiring wheel torque and is honoured only while the balance law is
//! disarmed. The balance path itself is smoothed with a short low-pass filter
//! on the pitch rate and a slew-limited torque command; the e-stop bypasses
//! both and zeroes the command immediately.

#![no_std]
#![no_main]

mod console;
mod imu;
mod state;

use core::sync::atomic::Ordering;

use defmt::info;
use defmt_rtt as _;
use embassy_executor::Spawner;
use embassy_rp::bind_interrupts;
use embassy_rp::gpio::{Level, Output};
use embassy_rp::i2c::I2c;
use embassy_rp::peripherals::{I2C1, UART0, USB};
use embassy_rp::uart::{BufferedInterruptHandler, BufferedUart, BufferedUartRx, BufferedUartTx};
use embassy_rp::watchdog::Watchdog;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::signal::Signal;
use embassy_time::{Duration, Instant, Ticker, Timer};
use embedded_io_async::{Read, Write};
use static_cell::StaticCell;

use jbot_pico_core::control::{
    to_command, BalanceController, BalanceInput, SafetyState, WheelTorque,
};
use jbot_pico_core::eferu::{Command, Feedback, FeedbackParser};
use jbot_pico_core::filters::{LowPass, SlewLimiter};
use jbot_pico_core::mount::{self, Mount};
use jbot_pico_core::reference::PitchReference;
use jbot_pico_core::spin::SpinTest;

use crate::imu::{Bno055, ImuSample};
use crate::state::Telemetry;

bind_interrupts!(struct Irqs {
    UART0_IRQ => BufferedInterruptHandler<UART0>;
    I2C1_IRQ => embassy_rp::i2c::InterruptHandler<I2C1>;
    USBCTRL_IRQ => embassy_rp::usb::InterruptHandler<USB>;
});

/// Control loop period: 5 ms, that is 200 Hz.
const CONTROL_PERIOD: Duration = Duration::from_micros(5_000);
/// Control loop period in seconds, for the rate limiters.
const CONTROL_PERIOD_S: f32 = 0.005;
/// Cut-off of the first-order low-pass filter on the pitch rate used by the
/// derivative term.
const D_FILTER_CUTOFF_HZ: f32 = 12.0;
/// Maximum rate of change of the applied wheel torque, command units per second.
const TORQUE_SLEW_PER_S: f32 = 3_000.0;
/// IMU sampling period: 10 ms, that is 100 Hz.
const IMU_PERIOD: Duration = Duration::from_millis(10);
/// An IMU sample older than this is not used for control.
const IMU_MAX_AGE: Duration = Duration::from_millis(50);
/// Mainboard watchdog period. Fed once per control step.
const WATCHDOG_PERIOD: Duration = Duration::from_millis(1_000);
/// UART link baud rate, matching the EFeru `VARIANT_USART` build.
const LINK_BAUD: u32 = 115_200;
/// BNO055 I2C clock.
const IMU_I2C_FREQ: u32 = 400_000;
/// Donor wheel radius, from `config/robot.toml`.
const WHEEL_RADIUS_M: f32 = 0.0825;
/// Absolute pitch above which the telemetry flags `MODE_RECOVER`. No control
/// behaviour changes; the flag exists so the operator sees that the robot is
/// well outside ordinary balance.
const RECOVER_LIMIT_RAD: f32 = core::f32::consts::PI / 12.0;
/// Wheel-speed feedback older than this freezes the position loop, and arming
/// is refused while it is older. The EFeru feedback frame arrives every
/// 100 ms, so the window has to be comfortably wider than one frame.
const FEEDBACK_MAX_AGE: Duration = Duration::from_millis(300);
/// Nominal pack voltage of the donor battery, volts.
const PACK_NOMINAL_V: f32 = 36.0;
/// Pose error below which arming anchors the reference to the measured pose,
/// radians (5 degrees). A larger error starts at the target so the law lifts
/// the body instead of slowly chasing it.
const ARM_ANCHOR_RAD: f32 = 5.0 * core::f32::consts::PI / 180.0;

/// Capacity of the buffered UART transmit ring.
const UART_TX_BUFFER_LEN: usize = 256;
/// Capacity of the buffered UART receive ring.
const UART_RX_BUFFER_LEN: usize = 256;

/// Latest IMU sample, overwritten at 100 Hz.
static IMU_SIGNAL: Signal<CriticalSectionRawMutex, ImuSample> = Signal::new();
/// Latest mainboard feedback frame.
static FEEDBACK_SIGNAL: Signal<CriticalSectionRawMutex, Feedback> = Signal::new();

static UART_TX_BUFFER: StaticCell<[u8; UART_TX_BUFFER_LEN]> = StaticCell::new();
static UART_RX_BUFFER: StaticCell<[u8; UART_RX_BUFFER_LEN]> = StaticCell::new();

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let p = embassy_rp::init(Default::default());
    info!("JBOT pico bridge starting");

    // Scope marker; also the hardware proof that the control loop runs.
    let marker = Output::new(p.PIN_10, Level::Low);

    let mut uart_config = embassy_rp::uart::Config::default();
    uart_config.baudrate = LINK_BAUD;
    let uart = BufferedUart::new(
        p.UART0,
        Irqs,
        // Pico TX -> STM32 PA3 (USART2 RX), Pico RX <- STM32 PA2 (USART2
        // TX). The wiring was reworked; do not share pins with the console.
        p.PIN_16,
        p.PIN_17,
        UART_TX_BUFFER.init([0u8; UART_TX_BUFFER_LEN]),
        UART_RX_BUFFER.init([0u8; UART_RX_BUFFER_LEN]),
        uart_config,
    );
    let (link_tx, link_rx) = uart.split();

    let mut i2c_config = embassy_rp::i2c::Config::default();
    i2c_config.frequency = IMU_I2C_FREQ;
    let i2c = I2c::new_async(p.I2C1, p.PIN_3, p.PIN_2, Irqs, i2c_config);
    let imu = Bno055::new(i2c, imu::ADDRESS);

    let watchdog = Watchdog::new(p.WATCHDOG);

    let usb_driver = embassy_rp::usb::Driver::new(p.USB, Irqs);
    let (usb, console) = console::new(usb_driver);

    spawner.must_spawn(console::usb_task(usb));
    spawner.must_spawn(console::console_task(console));
    spawner.must_spawn(imu_task(imu));
    spawner.must_spawn(link_rx_task(link_rx));
    spawner.must_spawn(control_task(link_tx, marker, watchdog));
}

/// Sample the BNO055 and publish the latest value.
#[embassy_executor::task]
async fn imu_task(mut imu: Bno055<'static, I2C1>) {
    loop {
        match imu.init().await {
            Ok(()) => {
                info!("BNO055 ready in IMUPLUS mode");
                break;
            }
            Err(error) => {
                state::count(&state::IMU_ERRORS);
                state::IMU_LAST_ERROR.store(error.code(), Ordering::Relaxed);
                info!("BNO055 init failed: {}, retrying", error);
                Timer::after_millis(1_000).await;
            }
        }
    }

    let mut ticker = Ticker::every(IMU_PERIOD);
    loop {
        ticker.next().await;

        if state::REINIT.try_take().is_some() {
            match imu.init().await {
                Ok(()) => info!("BNO055 re-initialised"),
                Err(error) => {
                    state::count(&state::IMU_ERRORS);
                    state::IMU_LAST_ERROR.store(error.code(), Ordering::Relaxed);
                    info!("BNO055 re-init failed: {}", error);
                }
            }
        }

        match imu.sample().await {
            Ok(sample) => {
                state::count(&state::IMU_SAMPLES);
                IMU_SIGNAL.signal(sample);
            }
            Err(error) => {
                state::count(&state::IMU_ERRORS);
                state::IMU_LAST_ERROR.store(error.code(), Ordering::Relaxed);
                defmt::warn!("BNO055 read failed: {}", error);
            }
        }
    }
}

/// Parse feedback frames from the STM32 torque node.
#[embassy_executor::task]
async fn link_rx_task(mut rx: BufferedUartRx<'static, UART0>) {
    let mut parser = FeedbackParser::new();
    let mut buffer = [0u8; 64];

    loop {
        match rx.read(&mut buffer).await {
            Ok(0) => {}
            Ok(len) => {
                state::LINK_BYTES.fetch_add(len as u32, Ordering::Relaxed);
                for &byte in &buffer[..len] {
                    if let Some(feedback) = parser.push(byte) {
                        state::count(&state::FB_FRAMES);
                        FEEDBACK_SIGNAL.signal(feedback);
                    }
                }
                state::FB_DISCARDED.store(parser.discarded_bytes, Ordering::Relaxed);
            }
            Err(error) => {
                defmt::warn!("link rx error: {}", error);
                Timer::after_millis(10).await;
            }
        }
    }
}

/// 200 Hz balance and torque-link task.
#[embassy_executor::task]
async fn control_task(
    mut tx: BufferedUartTx<'static, UART0>,
    mut marker: Output<'static>,
    mut watchdog: Watchdog,
) {
    watchdog.start(WATCHDOG_PERIOD);

    let base_tuning = state::tuning();
    let mut ticker = Ticker::every(CONTROL_PERIOD);
    let mut last_imu = ImuSample::default();
    let mut last_imu_at: Option<Instant> = None;
    let mut last_feedback = Feedback::default();
    let mut last_feedback_at: Option<Instant> = None;
    let mut spin = SpinTest::new();
    let mut rate_lp = LowPass::new(D_FILTER_CUTOFF_HZ, CONTROL_PERIOD_S);
    let mut torque_slew_left = SlewLimiter::new(TORQUE_SLEW_PER_S);
    let mut torque_slew_right = SlewLimiter::new(TORQUE_SLEW_PER_S);
    let mut balance = BalanceController::new();
    let mut reference = PitchReference::new();
    let mut was_armed = false;
    let mut steps: u32 = 0;
    // Travel since arming, metres, from the wheel-speed feedback.
    let mut position_m = 0.0_f32;
    let mut feedback_gap_logged = false;

    // Loop-rate measurement over a fixed window of control steps.
    let mut window_at = Instant::now();
    let mut window_steps: u32 = 0;

    loop {
        ticker.next().await;

        // Bench power scale, applied per tick so a console change takes
        // effect on the very next step. Only the angular-output gains move;
        // the fall guards see the floor, not the scaled gains.
        let tuning = base_tuning.scaled_by(state::pscale());

        if let Some(sample) = IMU_SIGNAL.try_take() {
            last_imu = sample;
            last_imu_at = Some(Instant::now());
            // Publish the mapped attitude on every sample, even while the
            // e-stop is latched, so the console captures live values.
            let attitude = attitude_of(&last_imu);
            state::set_latest_mapped_attitude(attitude.pitch_deg, attitude.roll_deg);
        }
        if let Some(feedback) = FEEDBACK_SIGNAL.try_take() {
            last_feedback = feedback;
            last_feedback_at = Some(Instant::now());
        }

        let imu_fresh = last_imu_at.is_some_and(|at| at.elapsed() < IMU_MAX_AGE);
        state::set_imu_fresh(imu_fresh);
        let feedback_fresh = last_feedback_at.is_some_and(|at| at.elapsed() < FEEDBACK_MAX_AGE);
        state::set_link_fresh(feedback_fresh);
        let measured_speed = if feedback_fresh {
            last_feedback.forward_speed_mps(WHEEL_RADIUS_M)
        } else {
            0.0
        };

        // The reference target follows the captured `level` by default; an
        // explicit `home` command overrides it. Latest write wins, like every
        // other setpoint.
        let target_rad = state::home_target_rad().unwrap_or_else(state::pitch_trim_rad);
        reference.set_target(target_rad);
        reference.set_v_max(state::ref_v_max_rad_s());

        // Per-tick flags; assigned on every branch of the command selection.
        let mut fell = false;
        let mut pitch_zero_rad: Option<f32> = None;
        let ref_moving: bool;
        let mut report: BalanceReport;

        // The bench spin test is a direct torque path that bypasses the
        // balance law; it is honoured only when the balance law is disarmed
        // and the e-stop is clear.
        let now_ms = Instant::now().as_millis();
        if let Some(pair) = state::SPIN_REQUEST.try_take() {
            spin.request(pair.left, pair.right, now_ms);
        }
        let estop = state::estop();
        let armed = state::armed();
        let spin_pair = spin.step(CONTROL_PERIOD_S, now_ms, armed, estop);
        state::set_spin_applied(spin_pair.left, spin_pair.right);

        let command = if estop {
            reset_smoothing(
                &mut rate_lp,
                &mut torque_slew_left,
                &mut torque_slew_right,
                &mut balance,
                &mut reference,
            );
            position_m = 0.0;
            state::set_balance_applied(0.0);
            ref_moving = false;
            report = BalanceReport::idle();
            Command::ZERO
        } else if armed {
            let attitude = attitude_of(&last_imu);
            let pitch_zero = attitude.pitch_deg.to_radians() - state::zero_pitch_rad();
            pitch_zero_rad = Some(pitch_zero);

            // Arming is refused when the torque link is blind, when the body
            // is already on the ground, or when the pose is outside the lift
            // budget. The decision itself is a pure, tested function; it
            // ignores travel while arming because arming resets the origin.
            let safety = SafetyState {
                pitch_rad: pitch_zero,
                reference_rad: target_rad,
                position_m,
                speed_mps: measured_speed,
                feedback_fresh,
            };
            let refusal = if was_armed {
                None
            } else {
                safety.arm_refusal(&tuning)
            };

            if let Some(reason) = refusal {
                fell = true;
                state::set_armed(false);
                state::set_stop(Some(reason));
                state::count(&state::FALL_EVENTS);
                defmt::warn!(
                    "balance refused: reason={} pitch={} err={} fb={}",
                    reason.as_str(),
                    pitch_zero,
                    safety.pose_error_rad(),
                    feedback_fresh
                );
                reset_smoothing(
                    &mut rate_lp,
                    &mut torque_slew_left,
                    &mut torque_slew_right,
                    &mut balance,
                    &mut reference,
                );
                position_m = 0.0;
                state::set_balance_applied(0.0);
                ref_moving = false;
                report = BalanceReport::idle();
                Command::ZERO
            } else if !imu_fresh {
                // No attitude sample: no torque. This is not latched, because
                // the mainboard stops on its own serial timeout and a single
                // sensor gap should not end the run.
                state::set_balance_applied(0.0);
                ref_moving = reference.velocity() != 0.0;
                report = BalanceReport::idle();
                Command::ZERO
            } else {
                if !was_armed {
                    // Arming soft-start. The reference is anchored to the
                    // measured pose only when the pose is already close to the
                    // target: anchoring a large error would cancel the control
                    // error and the machine would fall instead of being
                    // lifted.
                    if safety.pose_error_rad().abs() < ARM_ANCHOR_RAD {
                        reference.reset(pitch_zero);
                    } else {
                        reference.reset(target_rad);
                    }
                    position_m = 0.0;
                    balance.reset(0.0);
                }
                let step = reference.update(CONTROL_PERIOD_S);
                let applied_ref_rad = step.ref_rad;
                ref_moving = step.moving;
                let filtered_rate = rate_lp.update(attitude.pitch_rate_dps.to_radians());

                if feedback_fresh {
                    position_m += measured_speed * CONTROL_PERIOD_S;
                    feedback_gap_logged = false;
                } else if !feedback_gap_logged {
                    // Freeze the position loop and keep balancing on pitch: a
                    // link gap must not drop the machine.
                    defmt::warn!("feedback stale, position loop frozen");
                    feedback_gap_logged = true;
                }

                let input = BalanceInput {
                    pitch_rad: pitch_zero,
                    pitch_rate_rad_s: filtered_rate,
                    pitch_ref_rad: applied_ref_rad,
                    position_m,
                    speed_mps: measured_speed,
                    yaw_rate_error_rad_s: state::steer_setpoint_rps()
                        - attitude.yaw_rate_dps.to_radians(),
                    battery_scale: battery_scale(last_feedback.battery_voltage_v()),
                };
                let torque = balance.update(&input, &tuning, CONTROL_PERIOD_S);

                // Pose and range guards. A control error outside the lift
                // budget has to stop the attempt instead of feeding it, and a
                // runaway is aborted before it hurts anything.
                let guard = SafetyState {
                    reference_rad: applied_ref_rad,
                    position_m,
                    ..safety
                };
                if let Some(reason) = guard.fall_cause(&tuning) {
                    fell = true;
                    state::set_armed(false);
                    state::set_stop(Some(reason));
                    state::count(&state::FALL_EVENTS);
                    defmt::warn!(
                        "balance fall: reason={} err={} pitch={} x={} v={}",
                        reason.as_str(),
                        guard.pose_error_rad(),
                        pitch_zero,
                        position_m,
                        measured_speed
                    );
                    reset_smoothing(
                        &mut rate_lp,
                        &mut torque_slew_left,
                        &mut torque_slew_right,
                        &mut balance,
                        &mut reference,
                    );
                    position_m = 0.0;
                    state::set_balance_applied(0.0);
                    report = BalanceReport::idle();
                    Command::ZERO
                } else {
                    let left = torque_slew_left.update(torque.left, CONTROL_PERIOD_S);
                    let right = torque_slew_right.update(torque.right, CONTROL_PERIOD_S);
                    state::set_balance_applied(0.5 * (left + right));
                    report = BalanceReport {
                        ref_deg: applied_ref_rad.to_degrees(),
                        demand_deg: balance.demand_rad().to_degrees(),
                        trim_deg: balance.trim_rad().to_degrees(),
                        position_m,
                        mode: state::MODE_DISARMED,
                    };
                    to_command(WheelTorque { left, right })
                }
            }
        } else {
            reset_smoothing(
                &mut rate_lp,
                &mut torque_slew_left,
                &mut torque_slew_right,
                &mut balance,
                &mut reference,
            );
            position_m = 0.0;
            state::set_balance_applied(0.0);
            ref_moving = false;
            report = BalanceReport::idle();
            if spin_pair.left != 0.0 || spin_pair.right != 0.0 {
                to_command(WheelTorque {
                    left: spin_pair.left,
                    right: spin_pair.right,
                })
            } else {
                Command::ZERO
            }
        };

        // The mainboard expects exactly eight bytes between two idle gaps.
        let _ = tx.write_all(&command.encode()).await;

        marker.toggle();
        watchdog.feed();

        steps = steps.wrapping_add(1);
        state::count(&state::CONTROL_STEPS);

        window_steps += 1;
        if window_steps == state::CONTROL_HZ {
            let elapsed_us = window_at.elapsed().as_micros().max(1);
            let hz = (u64::from(window_steps) * 1_000_000 / elapsed_us) as u32;
            state::set_loop_hz(hz);
            window_at = Instant::now();
            window_steps = 0;
        }

        was_armed = state::armed();
        report.mode = if fell {
            state::MODE_FALL
        } else if was_armed {
            match pitch_zero_rad {
                Some(pitch) if pitch.abs() > RECOVER_LIMIT_RAD => state::MODE_RECOVER,
                _ if ref_moving => state::MODE_TRANSIT,
                _ => state::MODE_HOLD,
            }
        } else {
            state::MODE_DISARMED
        };

        // The console decides how often to print; it always sees the latest.
        let telemetry = snapshot(
            steps,
            &last_imu,
            imu_fresh,
            &last_feedback,
            last_feedback_at,
            &report,
        );
        state::TELEMETRY.signal(telemetry);

        if steps.is_multiple_of(state::CONTROL_HZ) {
            log_telemetry(&telemetry);
        }
    }
}

/// Balance outputs carried from the control step into the telemetry snapshot.
#[derive(Clone, Copy, Default)]
struct BalanceReport {
    /// Applied (ramped) pitch reference, degrees in the zeroed frame.
    ref_deg: f32,
    /// Total pitch offset demanded by the outer loops, degrees.
    demand_deg: f32,
    /// Learned slope trim, degrees.
    trim_deg: f32,
    /// Travel since arming, metres.
    position_m: f32,
    /// Balance mode code, filled in after the command selection.
    mode: u8,
}

impl BalanceReport {
    /// All-zero report for the paths that produce no balance torque.
    fn idle() -> Self {
        Self::default()
    }
}

/// Clear every smoothing stage so the balance law starts from zero on the
/// next arm instead of resuming a stale filter, limiter, integrator or
/// reference state.
fn reset_smoothing(
    rate_lp: &mut LowPass,
    torque_slew_left: &mut SlewLimiter,
    torque_slew_right: &mut SlewLimiter,
    balance: &mut BalanceController,
    reference: &mut PitchReference,
) {
    rate_lp.reset();
    torque_slew_left.reset();
    torque_slew_right.reset();
    balance.reset(0.0);
    reference.reset(0.0);
}

/// Torque scaling from the pack voltage.
///
/// The drive can deliver less torque at a lower pack voltage, so a sagging pack
/// gets a proportionally smaller torque budget. The law then saturates earlier
/// instead of asking for torque the pack cannot supply. A missing or
/// implausible reading leaves the scale neutral.
fn battery_scale(pack_v: f32) -> f32 {
    if pack_v > 10.0 {
        (pack_v / PACK_NOMINAL_V).clamp(0.5, 1.2)
    } else {
        1.0
    }
}

/// Derive the mount-compensated attitude of one sample.
fn attitude_of(sample: &ImuSample) -> mount::Attitude {
    mount::attitude(
        Mount::from_u32(state::mount()),
        sample.euler,
        sample.gyro,
        sample.gravity_ms2,
    )
}

/// Build the telemetry snapshot for this control step.
fn snapshot(
    steps: u32,
    sample: &ImuSample,
    imu_fresh: bool,
    feedback: &Feedback,
    feedback_at: Option<Instant>,
    report: &BalanceReport,
) -> Telemetry {
    let attitude = attitude_of(sample);
    let pitch_zero_deg = attitude.pitch_deg - state::zero_pitch_rad().to_degrees();
    let roll_zero_deg = attitude.roll_deg - state::zero_roll_rad().to_degrees();
    Telemetry {
        control_steps: steps,
        loop_hz: state::loop_hz(),
        imu_fresh,
        imu_calibrated: sample.imu_calibrated(),
        imu_system_cal: sample.calibration.system,
        pitch_deg: pitch_zero_deg,
        roll_deg: roll_zero_deg,
        heading_deg: sample.euler.heading_deg,
        gyro_y_dps: attitude.pitch_rate_dps,
        gyro_x_dps: attitude.roll_rate_dps,
        gyro_z_dps: attitude.yaw_rate_dps,
        sys_err: sample.sys_err,
        feedback_present: feedback_at.is_some(),
        speed_left_rpm: feedback.speed_left_rpm,
        speed_right_rpm: feedback.speed_right_rpm,
        battery_v: feedback.battery_voltage_v(),
        board_temp_c: feedback.board_temp_celsius(),
        feedback_age_ms: feedback_at.map_or(u32::MAX, |at| at.elapsed().as_millis() as u32),
        estop: state::estop(),
        speed_setpoint_mps: state::speed_setpoint_mps(),
        steer_setpoint_rps: state::steer_setpoint_rps(),
        armed: state::armed(),
        mount: state::mount() as u8,
        zero_offset_deg: state::zero_pitch_rad().to_degrees(),
        roll_zero_offset_deg: state::zero_roll_rad().to_degrees(),
        level_deg: state::pitch_trim_rad().to_degrees(),
        spin_left_units: state::spin_applied().left,
        spin_right_units: state::spin_applied().right,
        balance_torque_common_units: state::balance_applied(),
        pitch_ref_deg: report.ref_deg,
        pitch_demand_deg: report.demand_deg,
        slope_trim_deg: report.trim_deg,
        position_m: report.position_m,
        balance_mode: report.mode,
    }
}

/// One structured log line per second for a debug probe.
fn log_telemetry(telemetry: &Telemetry) {
    info!(
        "tick={} loop={} imu[{}] ready={} pitch={=f32} roll={=f32} heading={=f32} gyro_y={=f32} cal={} sys_err=0x{:x} | mainboard[{}] speed=({}, {}) rpm battery={=f32} V temp={=f32} C age={} ms estop={}",
        telemetry.control_steps,
        telemetry.loop_hz,
        if telemetry.imu_fresh { "ok" } else { "stale" },
        telemetry.imu_calibrated,
        telemetry.pitch_deg,
        telemetry.roll_deg,
        telemetry.heading_deg,
        telemetry.gyro_y_dps,
        telemetry.imu_system_cal,
        telemetry.sys_err,
        if telemetry.feedback_present { "ok" } else { "none" },
        telemetry.speed_left_rpm,
        telemetry.speed_right_rpm,
        telemetry.battery_v,
        telemetry.board_temp_c,
        telemetry.feedback_age_ms,
        telemetry.estop,
    );
}

#[panic_handler]
fn panic(_info: &core::panic::PanicInfo) -> ! {
    // The control task stops feeding the watchdog, so the chip resets itself
    // within one second instead of leaving stale torque commands in flight.
    defmt::error!("panic: firmware halted");
    loop {
        cortex_m::asm::wfi();
    }
}
