//! JBOT real-time controller firmware for the Raspberry Pi Pico 2 W (RP2350).
//!
//! This is the foundation stage of the Tier 1 controller described in
//! `docs/pico-bridge.md`: it brings up the clock tree, the UART link to the
//! STM32F103 torque node, the I2C link to the BNO055, the 200 Hz control tick,
//! and a USB console for bench work. The balance law exists but runs with
//! [`Gains::SAFE`], so every command frame carries zero torque until the robot
//! is tuned and the radio is fitted.
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
//! Safety: if the IMU sample is stale or missing, if the gains are zero, or if
//! the emergency stop is latched, the task sends [`Command::ZERO`]. The
//! mainboard independently stops on its own serial timeout.

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

use jbot_pico_core::control::{balance, to_command, BalanceInput, Gains, WheelTorque};
use jbot_pico_core::eferu::{Command, Feedback, FeedbackParser};

use crate::imu::{Bno055, ImuSample};
use crate::state::Telemetry;

bind_interrupts!(struct Irqs {
    UART0_IRQ => BufferedInterruptHandler<UART0>;
    I2C1_IRQ => embassy_rp::i2c::InterruptHandler<I2C1>;
    USBCTRL_IRQ => embassy_rp::usb::InterruptHandler<USB>;
});

/// Control loop period: 5 ms, that is 200 Hz.
const CONTROL_PERIOD: Duration = Duration::from_micros(5_000);
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

/// Balance gains. Zero until a bench tuning session produces real values.
const GAINS: Gains = Gains::SAFE;

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
        p.PIN_0,
        p.PIN_1,
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

    let mut ticker = Ticker::every(CONTROL_PERIOD);
    let mut last_imu = ImuSample::default();
    let mut last_imu_at: Option<Instant> = None;
    let mut last_feedback = Feedback::default();
    let mut last_feedback_at: Option<Instant> = None;
    let mut steps: u32 = 0;

    // Loop-rate measurement over a fixed window of control steps.
    let mut window_at = Instant::now();
    let mut window_steps: u32 = 0;

    loop {
        ticker.next().await;

        if let Some(sample) = IMU_SIGNAL.try_take() {
            last_imu = sample;
            last_imu_at = Some(Instant::now());
        }
        if let Some(feedback) = FEEDBACK_SIGNAL.try_take() {
            last_feedback = feedback;
            last_feedback_at = Some(Instant::now());
        }

        let imu_fresh = last_imu_at.is_some_and(|at| at.elapsed() < IMU_MAX_AGE);
        let command = if state::estop() {
            Command::ZERO
        } else {
            control_step(
                &last_imu,
                imu_fresh,
                &last_feedback,
                state::pitch_trim_rad(),
            )
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

        // The console decides how often to print; it always sees the latest.
        let telemetry = snapshot(
            steps,
            &last_imu,
            imu_fresh,
            &last_feedback,
            last_feedback_at,
        );
        state::TELEMETRY.signal(telemetry);

        if steps.is_multiple_of(state::CONTROL_HZ) {
            log_telemetry(&telemetry);
        }
    }
}

/// Compute one command frame from the latest sample and feedback.
///
/// Torque is produced only when the sample is fresh *and* the gains are
/// non-zero. Everything else yields a zero command.
///
/// Axis mapping assumes the BNO055 X axis points forward, Y left and Z up, so
/// pitch is the Euler pitch and the pitch rate is gyro Y. Verify the signs on
/// the bench before enabling non-zero gains; the module is mounted by hand.
fn control_step(
    sample: &ImuSample,
    imu_fresh: bool,
    feedback: &Feedback,
    pitch_trim_rad: f32,
) -> Command {
    if !imu_fresh || !GAINS.is_active() {
        return Command::ZERO;
    }

    let (left_mps, right_mps) = feedback.wheel_speed_mps(WHEEL_RADIUS_M);
    let measured_speed = 0.5 * (left_mps + right_mps);

    let input = BalanceInput {
        pitch_rad: sample.euler.pitch_deg.to_radians() - pitch_trim_rad,
        pitch_rate_rad_s: sample.gyro.y_dps.to_radians(),
        // The setpoint is zero until the radio link exists.
        speed_error_mps: -measured_speed,
        yaw_rate_error_rad_s: -sample.gyro.z_dps.to_radians(),
    };

    let torque: WheelTorque = balance(&input, &GAINS);
    to_command(torque)
}

/// Build the telemetry snapshot for this control step.
fn snapshot(
    steps: u32,
    sample: &ImuSample,
    imu_fresh: bool,
    feedback: &Feedback,
    feedback_at: Option<Instant>,
) -> Telemetry {
    Telemetry {
        control_steps: steps,
        loop_hz: state::loop_hz(),
        imu_fresh,
        imu_calibrated: sample.imu_calibrated(),
        imu_system_cal: sample.calibration.system,
        pitch_deg: sample.euler.pitch_deg,
        gyro_y_dps: sample.gyro.y_dps,
        sys_err: sample.sys_err,
        feedback_present: feedback_at.is_some(),
        speed_left_rpm: feedback.speed_left_rpm,
        speed_right_rpm: feedback.speed_right_rpm,
        battery_v: feedback.battery_voltage_v(),
        board_temp_c: feedback.board_temp_celsius(),
        feedback_age_ms: feedback_at.map_or(u32::MAX, |at| at.elapsed().as_millis() as u32),
        estop: state::estop(),
    }
}

/// One structured log line per second for a debug probe.
fn log_telemetry(telemetry: &Telemetry) {
    info!(
        "tick={} loop={} imu[{}] ready={} pitch={=f32} gyro_y={=f32} cal={} sys_err=0x{:x} | mainboard[{}] speed=({}, {}) rpm battery={=f32} V temp={=f32} C age={} ms estop={}",
        telemetry.control_steps,
        telemetry.loop_hz,
        if telemetry.imu_fresh { "ok" } else { "stale" },
        telemetry.imu_calibrated,
        telemetry.pitch_deg,
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
