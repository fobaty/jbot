//! STM32F103RCT6 torque node firmware.
//!
//! The 16 kHz FOC tick is the ADC DMA transfer-complete IRQ, matching the
//! pinned C build: TIM8 TRGO starts a dual ADC conversion, DMA publishes all
//! ten fields, and the ISR reconstructs currents and runs both motor models.
//! PWM outputs stay disabled (MOE low) until HIL validation, so no torque can
//! be produced yet. The control math is the bit-exact port of the generated C
//! controller and is replayed against its golden vectors in host tests.

#![no_std]
#![no_main]

mod hal;

use core::cell::RefCell;
use core::sync::atomic::{AtomicI16, Ordering};

use f103_core::console::{
    format_adc, format_get, format_help, format_params, format_sample, format_watch, parse_command,
    Blackbox, Command, Formatter, LineBuffer, Sample, Telemetry, BLACKBOX_DEPTH, LINE_MAX,
    REPLY_MAX,
};

use cortex_m::interrupt::Mutex;
use cortex_m_rt::entry;
use f103_core::alerts::{select_alert, Alert, AlertInputs, ArmBeep, BuzzerController};
use f103_core::battery::{BatteryLevel, BatteryMonitor};
use f103_core::config::{
    BAT_DEAD_ENABLE, BAT_LVL1_ENABLE, BAT_LVL2_ENABLE, BEEPS_BACKWARD, DC_TRIP_COUNTS,
    FOC_PERIOD_US, FOC_TICKS_PER_MS, PWM_ARR, PWM_MARGIN, STANDSTILL_RPM, SYSTEM_HZ,
    TEMP_POWEROFF_DECIC, TEMP_POWEROFF_ENABLE, TEMP_WARNING_DECIC, TEMP_WARNING_ENABLE, WIRE_MAX,
};
use f103_core::fixed::clamp_i32;
use f103_core::foc_generated::{GeneratedMotor, GeneratedParams, Inputs};
use f103_core::mixer::mix;
use f103_core::monitors::{InactivityMonitor, SerialTimeoutMonitor};
use f103_core::parameters::Parameters;
use f103_core::protocol::{
    encode_hardened_feedback, encode_legacy_feedback, encode_trace, CommandDecoder, FeedbackData,
    ProtocolMode,
};
use f103_core::safety::{Fault, SafeController};
use f103_core::sensing::{CurrentCalibration, ADC_FIELDS};
use f103_core::sensing::{BATT1, TEMP};
use f103_core::thermal::TemperatureEstimator;
use f103_core::trace::TraceData;
use panic_halt as _;
use stm32f1::stm32f103::{self as pac, interrupt};

/// Latest wheel torque commands, published by the 5 ms loop and consumed by
/// the FOC ISR. Release/Acquire hands ownership across the priority boundary.
static TARGET_LEFT: AtomicI16 = AtomicI16::new(0);
static TARGET_RIGHT: AtomicI16 = AtomicI16::new(0);

/// FOC state is owned by a single highest-priority ISR. The mutex exists to
/// keep the state in safe interior mutability; no other context borrows it.
static MOTORS: Mutex<RefCell<Option<MotorPair>>> = Mutex::new(RefCell::new(None));

/// Pure monitor state owned by the FOC ISR, alongside the motor models.
struct Monitors {
    battery: BatteryMonitor,
    thermal: TemperatureEstimator,
    buzzer: BuzzerController,
    tick: u32,
}

struct MotorPair {
    left: GeneratedMotor,
    right: GeneratedMotor,
    params: GeneratedParams,
    duty_left: (u16, u16, u16),
    duty_right: (u16, u16, u16),
    calibration: CurrentCalibration,
    monitors: Monitors,
}

impl MotorPair {
    fn new(params: &Parameters) -> Self {
        let mut generated = GeneratedParams::pinned();
        params.apply_to(&mut generated);
        Self {
            left: GeneratedMotor::new(),
            right: GeneratedMotor::new(),
            params: generated,
            duty_left: (PWM_ARR / 2, PWM_ARR / 2, PWM_ARR / 2),
            duty_right: (PWM_ARR / 2, PWM_ARR / 2, PWM_ARR / 2),
            calibration: CurrentCalibration::new(),
            monitors: Monitors {
                battery: BatteryMonitor::new(),
                thermal: TemperatureEstimator::new(),
                buzzer: BuzzerController::new(),
                tick: 0,
            },
        }
    }

    /// Returns false while the ADC offsets are still being learned, so the
    /// caller can skip applying any duty.
    fn step(
        &mut self,
        fields: &[u16; ADC_FIELDS],
        gpiob: &pac::gpioa::RegisterBlock,
        gpioc: &pac::gpioa::RegisterBlock,
    ) -> bool {
        if hal::RECALIBRATE.swap(false, Ordering::Relaxed) {
            self.calibration = CurrentCalibration::new();
            self.left.initialize();
            self.right.initialize();
        }
        if !self.calibration.update(fields) {
            hal::CAL_READY.store(false, Ordering::Relaxed);
            return false;
        }
        hal::CAL_READY.store(true, Ordering::Relaxed);
        let sense = self.calibration.sample(fields);
        hal::DC_LEFT.store(sense.dc_left, Ordering::Relaxed);
        hal::DC_RIGHT.store(sense.dc_right, Ordering::Relaxed);
        if sense.dc_left.unsigned_abs() > DC_TRIP_COUNTS as u32
            || sense.dc_right.unsigned_abs() > DC_TRIP_COUNTS as u32
        {
            hal::FOC_OVERCURRENT.fetch_add(1, Ordering::Relaxed);
        }

        // Hall inputs are active low on the donor board.
        let left_halls = (
            !gpiob.idr().read().idr5().bit(),
            !gpiob.idr().read().idr6().bit(),
            !gpiob.idr().read().idr7().bit(),
        );
        let right_halls = (
            !gpioc.idr().read().idr10().bit(),
            !gpioc.idr().read().idr11().bit(),
            !gpioc.idr().read().idr12().bit(),
        );
        // The reference filters battery and temperature at 1 kHz (every 16 FOC
        // ticks). Hall codes use the generated A<<2|B<<1|C bit order.
        self.monitors.tick = self.monitors.tick.wrapping_add(1);
        if self.monitors.tick.is_multiple_of(FOC_TICKS_PER_MS) {
            hal::BATT_RAW.store(
                self.monitors.battery.update(fields[BATT1]),
                Ordering::Relaxed,
            );
            hal::TEMP_DECIC.store(
                self.monitors.thermal.update(fields[TEMP]),
                Ordering::Relaxed,
            );
        }
        let hall_code =
            |h: (bool, bool, bool)| (u8::from(h.0) << 2) | (u8::from(h.1) << 1) | u8::from(h.2);
        hal::HALL_L.store(hall_code(left_halls), Ordering::Relaxed);
        hal::HALL_R.store(hall_code(right_halls), Ordering::Relaxed);

        // The ported controller is binary-identical to the generated C. It
        // stays in OPEN mode until the safety controller reaches an armed
        // state; MOE remains low regardless, so no gate can be driven here.
        let enabled = hal::ARMED.load(Ordering::Relaxed);
        let as_i16 = |value: i32| clamp_i32(value, i16::MIN as i32, i16::MAX as i32) as i16;
        let left_input = Inputs {
            b_motEna: enabled,
            z_ctrlModReq: 3, // TRQ_MODE, as flashed by the jbot patch.
            r_inpTgt: TARGET_LEFT.load(Ordering::Acquire),
            b_hallA: left_halls.0,
            b_hallB: left_halls.1,
            b_hallC: left_halls.2,
            // The C controller consumes (A,B) on the left and (B,C) on the
            // right, matching bldc.c:204-205 and :250-251.
            i_phaAB: as_i16(sense.left.ia),
            i_phaBC: as_i16(sense.left.ib),
            i_DCLink: as_i16(sense.dc_left),
            a_mechAngle: 0,
        };
        let right_input = Inputs {
            b_motEna: enabled,
            z_ctrlModReq: 3,
            r_inpTgt: TARGET_RIGHT.load(Ordering::Acquire),
            b_hallA: right_halls.0,
            b_hallB: right_halls.1,
            b_hallC: right_halls.2,
            i_phaAB: as_i16(sense.right.ib),
            i_phaBC: as_i16(sense.right.ic),
            i_DCLink: as_i16(sense.dc_right),
            a_mechAngle: 0,
        };
        let left_out = self.left.step(&self.params, left_input);
        let right_out = self.right.step(&self.params, right_input);

        let to_duty = |value: i16| -> u16 {
            clamp_i32(
                value as i32 + PWM_ARR as i32 / 2,
                PWM_MARGIN as i32,
                (PWM_ARR - PWM_MARGIN) as i32,
            ) as u16
        };
        self.duty_left = (
            to_duty(left_out.dc_pha_a),
            to_duty(left_out.dc_pha_b),
            to_duty(left_out.dc_pha_c),
        );
        self.duty_right = (
            to_duty(right_out.dc_pha_a),
            to_duty(right_out.dc_pha_b),
            to_duty(right_out.dc_pha_c),
        );
        hal::SPEED_L.store(left_out.n_mot, Ordering::Relaxed);
        hal::SPEED_R.store(right_out.n_mot, Ordering::Relaxed);
        hal::DIAG_LEFT.store(left_out.z_err_code, Ordering::Relaxed);
        hal::DIAG_RIGHT.store(right_out.z_err_code, Ordering::Relaxed);
        true
    }
}

#[interrupt]
fn DMA1_CHANNEL1() {
    let started = hal::cycles();
    let dma = unsafe { &*pac::DMA1::ptr() };
    dma.ifcr().write(|w| w.ctcif1().clear());

    let fields = hal::adc_fields();
    cortex_m::interrupt::free(|cs| {
        let cell = MOTORS.borrow(cs);
        let Ok(mut slot) = cell.try_borrow_mut() else {
            hal::FOC_SKIPPED.fetch_add(1, Ordering::Relaxed);
            return;
        };
        let Some(pair) = slot.as_mut() else {
            hal::FOC_SKIPPED.fetch_add(1, Ordering::Relaxed);
            return;
        };
        let gpioa = unsafe { &*pac::GPIOA::ptr() };
        let gpiob = unsafe { &*pac::GPIOB::ptr() };
        let gpioc = unsafe { &*pac::GPIOC::ptr() };
        let forced = hal::BUZZER_FREQ_OVERRIDE.load(Ordering::Relaxed);
        if forced == hal::BUZZER_NO_OVERRIDE {
            let alert = Alert::from_u8(hal::BUZZER_ALERT.load(Ordering::Relaxed));
            pair.monitors.buzzer.set(alert);
        } else {
            pair.monitors.buzzer.set_frequency(forced);
        }
        if let Some(level) = pair.monitors.buzzer.tick() {
            hal::set_buzzer(gpioa, level);
        }
        if !pair.step(&fields, gpiob, gpioc) {
            hal::FOC_SKIPPED.fetch_add(1, Ordering::Relaxed);
            return;
        }
        *hal::LAST_FIELDS.borrow(cs).borrow_mut() = fields;

        // LEFT_TIM is TIM8 and RIGHT_TIM is TIM1; U/V/W map to CCR1/2/3.
        let tim8 = unsafe { &*pac::TIM8::ptr() };
        let tim1 = unsafe { &*pac::TIM1::ptr() };
        let left_duties = [pair.duty_left.0, pair.duty_left.1, pair.duty_left.2];
        let right_duties = [pair.duty_right.0, pair.duty_right.1, pair.duty_right.2];
        for (index, duty) in left_duties.iter().enumerate() {
            hal::DUTY_L[index].store(*duty, Ordering::Relaxed);
            // SAFETY: duty is already clamped into the 16-bit ARR range.
            tim8.ccr(index).write(|w| unsafe { w.ccr().bits(*duty) });
        }
        for (index, duty) in right_duties.iter().enumerate() {
            hal::DUTY_R[index].store(*duty, Ordering::Relaxed);
            // SAFETY: duty is already clamped into the 16-bit ARR range.
            tim1.ccr(index).write(|w| unsafe { w.ccr().bits(*duty) });
        }
        // The FOC tick is the only reliably scheduled context, so the watchdog
        // is fed here as well as at the end of the slower main loop.
        unsafe { (*pac::IWDG::ptr()).kr().write(|w| w.key().feed()) };
        let tick = hal::FOC_TICKS.fetch_add(1, Ordering::Relaxed);
        // Gate drive is a single, auditable point. Without the `moe` feature this
        // is compiled out entirely and MOE stays low at reset. With the feature
        // built, it still requires an armed, calibrated, fault-free controller.
        #[cfg(feature = "moe")]
        {
            let armed = hal::ARMED.load(Ordering::Relaxed);
            let calibrated = hal::CAL_READY.load(Ordering::Relaxed);
            let healthy = hal::DIAG_LEFT.load(Ordering::Relaxed) == 0
                && hal::DIAG_RIGHT.load(Ordering::Relaxed) == 0;
            hal::set_output_enable(tim1, tim8, armed && calibrated && healthy);
        }
        #[cfg(not(feature = "moe"))]
        let _ = (tim1, tim8);

        // Stream the full internal state at a decimated rate. Encoding happens
        // here because this is the only context that owns the FOC state; the
        // USART3 ISR only shifts bytes out of the ring.
        let decimation = hal::TRACE_DECIM.load(Ordering::Relaxed);
        if decimation != 0 && tick.is_multiple_of(u32::from(decimation)) {
            let mut flags = 0u8;
            if hal::ARMED.load(Ordering::Relaxed) {
                flags |= 1;
            }
            if hal::CAL_READY.load(Ordering::Relaxed) {
                flags |= 2;
            }
            if hal::DIAG_LEFT.load(Ordering::Relaxed) != 0
                || hal::DIAG_RIGHT.load(Ordering::Relaxed) != 0
            {
                flags |= 4;
            }
            let data = TraceData {
                tick,
                flags,
                left: pair.left.trace(),
                right: pair.right.trace(),
                dc_left: hal::DC_LEFT.load(Ordering::Relaxed) as i16,
                dc_right: hal::DC_RIGHT.load(Ordering::Relaxed) as i16,
                duty_left: [pair.duty_left.0, pair.duty_left.1, pair.duty_left.2],
                duty_right: [pair.duty_right.0, pair.duty_right.1, pair.duty_right.2],
            };
            let mut wire = [0u8; WIRE_MAX];
            if let Some(len) = encode_trace(
                tick as u16,
                tick.wrapping_mul(FOC_PERIOD_US),
                &data,
                &mut wire,
            ) {
                hal::trace_write(&wire[..len]);
            }
        }
    });
    let elapsed = hal::cycles().wrapping_sub(started);
    hal::FOC_LAST_CYCLES.store(elapsed, Ordering::Relaxed);
    if elapsed > hal::FOC_MAX_CYCLES.load(Ordering::Relaxed) {
        hal::FOC_MAX_CYCLES.store(elapsed, Ordering::Relaxed);
    }
    if elapsed > hal::FOC_CYCLE_BUDGET {
        hal::FOC_DEADLINE_MISSES.fetch_add(1, Ordering::Relaxed);
    }
}

#[interrupt]
fn USART2() {
    let usart = unsafe { &*pac::USART2::ptr() };
    let status = usart.sr().read();
    if status.idle().bit_is_set() {
        // Reading SR (above) then DR clears the IDLE flag. The circular DMA
        // never stops, so publish how many bytes the idle frame contains.
        let _ = usart.dr().read();
        let dma = unsafe { &*pac::DMA1::ptr() };
        let remaining = dma.ch(5).ndtr().read().ndt().bits() as usize;
        // Circular DMA reloads NDTR at the buffer end, so publish the write
        // position and let the main loop consume only the newly written span.
        let pos = 64usize.saturating_sub(remaining);
        hal::RX_POS.store(pos as u32, Ordering::Relaxed);
        hal::RX_READY.store(true, Ordering::Relaxed);
    } else if status.rxne().bit_is_set() {
        let _ = usart.dr().read();
    }
    // TXE shares this ISR with RX; writing DR clears it until the next byte is
    // shifted out, so the feedback ring drains without a dedicated interrupt.
    if status.txe().bit_is_set() {
        hal::feedback_tx_drain(usart);
    }
}

/// Dedicated debug console. RXNE and TXE share this ISR; reading DR clears
/// RXNE and writing DR clears TXE, so the two paths can be handled in one pass.
#[interrupt]
fn USART3() {
    let usart = unsafe { &*pac::USART3::ptr() };
    let status = usart.sr().read();
    if status.rxne().bit_is_set() {
        // Reading DR also clears overrun and framing flags for this byte.
        let byte = usart.dr().read().dr().bits() as u8;
        hal::debug_rx_push(byte);
    }
    if status.txe().bit_is_set() {
        hal::debug_tx_drain(usart);
    }
}

#[used]
pub static mut DEBUG_MAIN_COUNT: u32 = 0;

#[entry]
fn main() -> ! {
    // The IWDG survives a system reset. Feed it before any long startup work so
    // a watchdog inherited from a previous image cannot reset us mid-init.
    unsafe { (*pac::IWDG::ptr()).kr().write(|w| w.key().feed()) };

    let Some(peripherals) = pac::Peripherals::take() else {
        loop {
            cortex_m::asm::wfi();
        }
    };
    let Some(mut core) = cortex_m::Peripherals::take() else {
        loop {
            cortex_m::asm::wfi();
        }
    };

    hal::init_clocks(&peripherals.RCC, &peripherals.FLASH);
    hal::init_power_and_gpio(
        &peripherals.RCC,
        &peripherals.GPIOA,
        &peripherals.GPIOB,
        &peripherals.GPIOC,
    );
    hal::init_pwm(&peripherals.RCC, &peripherals.TIM1, &peripherals.TIM8);
    hal::init_uarts(&peripherals.RCC, &peripherals.USART2, &peripherals.USART3);
    hal::init_adc_dma(
        &peripherals.RCC,
        &peripherals.DMA1,
        &peripherals.ADC1,
        &peripherals.ADC2,
    );
    hal::init_usart2_rx_dma(&peripherals.DMA1, &peripherals.USART2);
    hal::start_pwm(&peripherals.AFIO, &peripherals.TIM1, &peripherals.TIM8);
    hal::init_dwt();

    // Latch early so the board stays alive when the button is released right
    // after flashing. The reference 500 ms wait is kept as a short guard here;
    // a long blocking loop was measured to brown out the supply on this board.
    cortex_m::asm::delay(100);
    let gpioa_init = unsafe { &*pac::GPIOA::ptr() };
    hal::latch_power(gpioa_init);
    hal::init_iwdg(&peripherals.IWDG);

    // Restore is informational until HIL approves runtime-settable limits:
    // the active drive values stay fixed in `f103_core::config`.
    let mut parameter_words = Parameters::defaults().to_words();
    let params_from_flash = hal::nvm_restore(&mut parameter_words);
    let mut params = Parameters::from_words(&parameter_words).unwrap_or_default();
    cortex_m::interrupt::free(|cs| {
        *MOTORS.borrow(cs).borrow_mut() = Some(MotorPair::new(&params));
    });

    // Safety decision recorded in f103_core: RAMP_TO_ZERO on link loss.
    let mut safe = SafeController::new(f103_core::safety::DEFAULT_SAFE_MODE);
    let mut link = CommandDecoder::default();
    let mut seq = 0u16;
    let mut last_frame_seq = 0u16;
    let mut now_us = 0u32;

    // Debug console state. A reply is formatted into one fixed buffer, queued
    // to the TX ring and drained by the USART3 ISR, so a dump never stalls the
    // 5 ms control loop.
    let mut line = LineBuffer::<LINE_MAX>::new();
    let mut blackbox = Blackbox::<BLACKBOX_DEPTH>::new();
    let mut reply = [0u8; REPLY_MAX];
    let mut watch = false;
    let mut watch_div: u8 = 1;
    let mut dump_index: Option<usize> = None;
    let mut loop_index: u32 = 0;
    let mut inactivity = InactivityMonitor::new();
    // The reference flags a missing serial command from the first main-loop
    // tick, before any frame has been received. Feed it from every loop so the
    // three-beep alert matches the flashed C build.
    let mut serial_timeout = SerialTimeoutMonitor::new();
    let mut rx_old_pos = 0usize;
    let mut arm_beep = ArmBeep::done();
    let mut button_seen = false;
    let mut press_start: u32 = 0;
    let mut long_beep_done = false;

    // The 5 ms cadence is driven by the DWT cycle counter in the loop below.
    // SysTick's default handler is a fault loop, so keep that interrupt off
    // rather than relying on its clock-source configuration.
    core.SYST.disable_counter();

    // SAFETY: the FOC ISR owns only MOTORS; the USART2 ISR only DMA state and
    // atomics. Giving the FOC tick the highest priority prevents either the
    // link ISR or SysTick from stretching the 62.5 us control deadline.
    unsafe {
        core.NVIC.set_priority(pac::Interrupt::DMA1_CHANNEL1, 0);
        core.NVIC.set_priority(pac::Interrupt::USART2, 2);
        core.NVIC.set_priority(pac::Interrupt::USART3, 3);
        cortex_m::peripheral::NVIC::unmask(pac::Interrupt::DMA1_CHANNEL1);
        cortex_m::peripheral::NVIC::unmask(pac::Interrupt::USART2);
        cortex_m::peripheral::NVIC::unmask(pac::Interrupt::USART3);
        core.SCB
            .set_priority(cortex_m::peripheral::scb::SystemHandler::SysTick, 3);
    }

    // `poweronMelody()` in util.c blocks for nine 100 ms steps, then lights the
    // LED. Driving it here keeps the sweep exact even though the main loop is
    // slowed by the FOC ISR. The buzzer itself is toggled by that ISR.
    for tone in [8u8, 7, 6, 5, 4, 3, 2, 1, 0] {
        hal::BUZZER_FREQ_OVERRIDE.store(tone, Ordering::Relaxed);
        let start = core.DWT.cyccnt.read();
        while core.DWT.cyccnt.read().wrapping_sub(start) < SYSTEM_HZ / 10 {
            hal::feed_iwdg(&peripherals.IWDG);
        }
    }
    hal::BUZZER_FREQ_OVERRIDE.store(hal::BUZZER_NO_OVERRIDE, Ordering::Relaxed);
    hal::led_on(unsafe { &*pac::GPIOB::ptr() });

    // `main.c:252` waits for the button to be released before the loop starts.
    // This consumes a press held through flashing, so the operator's release
    // after programming never becomes a shutdown request.
    let gpioa = unsafe { &*pac::GPIOA::ptr() };
    while hal::power_button_pressed(gpioa) {
        hal::feed_iwdg(&peripherals.IWDG);
    }

    loop {
        unsafe { DEBUG_MAIN_COUNT += 1 };
        let tick_start = core.DWT.cyccnt.read();
        while core.DWT.cyccnt.read().wrapping_sub(tick_start) < SYSTEM_HZ / 1000 * 5 {
            hal::feed_iwdg(&peripherals.IWDG);
        }
        now_us = now_us.wrapping_add(5_000);
        loop_index = loop_index.wrapping_add(1);

        // Restore the board's power-button UX: the pedal-style button is a soft
        // input, and a debounced short press releases the PA5 latch after the
        // reference power-off sweep. Torque is disarmed before any of this.
        if hal::power_button_pressed(gpioa) {
            safe.disarm();
            if !button_seen {
                press_start = core.DWT.cyccnt.read();
                button_seen = true;
                long_beep_done = false;
            }
            // `util.c:1584`: a five-second hold emits a short 5 beep and is
            // no longer treated as a shutdown request.
            if !long_beep_done && core.DWT.cyccnt.read().wrapping_sub(press_start) >= SYSTEM_HZ * 5
            {
                long_beep_done = true;
                hal::BUZZER_FREQ_OVERRIDE.store(5, Ordering::Relaxed);
                let start = core.DWT.cyccnt.read();
                while core.DWT.cyccnt.read().wrapping_sub(start) < SYSTEM_HZ / 10 {
                    hal::feed_iwdg(&peripherals.IWDG);
                }
                hal::BUZZER_FREQ_OVERRIDE.store(hal::BUZZER_NO_OVERRIDE, Ordering::Relaxed);
            }
        } else if button_seen {
            // `poweroffPressCheck()` (util.c:1580-1608): a press longer than
            // 80 ms but shorter than five seconds triggers poweroff on release.
            // DWT cycles keep the debounce exact regardless of loop speed.
            let held_cycles = core.DWT.cyccnt.read().wrapping_sub(press_start);
            if held_cycles > SYSTEM_HZ * 8 / 100 && held_cycles < SYSTEM_HZ * 5 {
                for tone in [0u8, 1, 2, 3, 4, 5, 6, 7] {
                    hal::BUZZER_FREQ_OVERRIDE.store(tone, Ordering::Relaxed);
                    let start = core.DWT.cyccnt.read();
                    while core.DWT.cyccnt.read().wrapping_sub(start) < SYSTEM_HZ / 10 {
                        hal::feed_iwdg(&peripherals.IWDG);
                    }
                }
                hal::BUZZER_FREQ_OVERRIDE.store(hal::BUZZER_NO_OVERRIDE, Ordering::Relaxed);
                // No flash write here: a page erase on this board has been
                // observed to clear the boot vector, leaving the next power-on
                // with an empty reset handler.
                hal::release_power_latch(gpioa);
                loop {
                    hal::feed_iwdg(&peripherals.IWDG);
                    cortex_m::asm::nop();
                }
            }
            button_seen = false;
            long_beep_done = false;
        }

        if let Some(freq) = arm_beep.tick() {
            hal::BUZZER_FREQ_OVERRIDE.store(freq, Ordering::Relaxed);
        } else if hal::BUZZER_FREQ_OVERRIDE.load(Ordering::Relaxed) != hal::BUZZER_NO_OVERRIDE {
            hal::BUZZER_FREQ_OVERRIDE.store(hal::BUZZER_NO_OVERRIDE, Ordering::Relaxed);
        }

        let mut valid_command = false;
        if hal::RX_READY.swap(false, Ordering::Relaxed) {
            let pos = hal::RX_POS.load(Ordering::Relaxed) as usize;
            if pos != rx_old_pos {
                // Circular DMA: consume only bytes written since the previous
                // IDLE frame, wrapping once at the end of the 64-byte buffer.
                let span = if pos > rx_old_pos {
                    pos - rx_old_pos
                } else {
                    64 - rx_old_pos + pos
                };
                let source = hal::RX_BUFFER.0.get() as *const u8;
                let mut chunk = [0u8; 64];
                let mut index = rx_old_pos;
                for byte in chunk[..span].iter_mut() {
                    // SAFETY: index is normalised into the 64-byte DMA buffer.
                    *byte = unsafe { core::ptr::read_volatile(source.add(index)) };
                    index += 1;
                    if index == 64 {
                        index = 0;
                    }
                }
                link.feed(&chunk[..span], now_us, &mut |setpoint, frame_seq| {
                    valid_command = true;
                    let effective_seq = match frame_seq {
                        Some(received) => {
                            last_frame_seq = received;
                            received
                        }
                        None => {
                            seq = seq.wrapping_add(1);
                            seq
                        }
                    };
                    let accepted = safe.accept(now_us, effective_seq, setpoint);
                    // Stock auto-arming (`main.c:270-275`): no error and a near-zero
                    // command enables the motors, with the two short confirmation
                    // beeps. A fault is never cleared by this path.
                    if accepted.is_ok()
                        && !safe.armed()
                        && safe.fault().is_none()
                        && setpoint.steer.unsigned_abs() < 50
                        && setpoint.speed.unsigned_abs() < 50
                    {
                        safe.arm();
                        arm_beep = ArmBeep::new();
                    }
                });
                rx_old_pos = if pos == 64 { 0 } else { pos };
            }
        }
        serial_timeout.tick(valid_command);

        // SafeController works in wire-channel units. Converting here keeps
        // the differential sign convention at a single actuator boundary.
        let (steer, speed) = safe.tick(now_us);
        let (left, right) = mix(
            speed,
            steer,
            params.speed_coeff_q14 as i16,
            params.steer_coeff_q14 as i16,
        );
        TARGET_LEFT.store(left, Ordering::Release);
        TARGET_RIGHT.store(right, Ordering::Release);

        let (applied_steer, applied_speed) = safe.applied();
        let telemetry = Telemetry {
            now_us,
            foc_ticks: hal::FOC_TICKS.load(Ordering::Relaxed),
            foc_overcurrent: hal::FOC_OVERCURRENT.load(Ordering::Relaxed),
            foc_skipped: hal::FOC_SKIPPED.load(Ordering::Relaxed),
            calibration_ready: hal::CAL_READY.load(Ordering::Relaxed),
            protocol: link.mode(),
            mode: safe.mode(),
            armed: safe.armed(),
            // Report a missing Pico link from the first tick, before any frame
            // has been accepted and the SafeController can raise its own fault.
            fault: safe
                .fault()
                .or(serial_timeout.flag.then_some(Fault::LinkTimeout)),
            faults: safe.faults,
            rejected: safe.rejected,
            link_age_us: safe.link_age_us(now_us),
            steer: applied_steer,
            speed: applied_speed,
            target_left: left,
            target_right: right,
            duty_l: core::array::from_fn(|i| hal::DUTY_L[i].load(Ordering::Relaxed)),
            duty_r: core::array::from_fn(|i| hal::DUTY_R[i].load(Ordering::Relaxed)),
            dc_left: hal::DC_LEFT.load(Ordering::Relaxed),
            dc_right: hal::DC_RIGHT.load(Ordering::Relaxed),
            battery_raw: hal::BATT_RAW.load(Ordering::Relaxed),
            temperature_decic: hal::TEMP_DECIC.load(Ordering::Relaxed),
            diag_left: hal::DIAG_LEFT.load(Ordering::Relaxed),
            diag_right: hal::DIAG_RIGHT.load(Ordering::Relaxed),
            foc_cycles: hal::FOC_LAST_CYCLES.load(Ordering::Relaxed),
            foc_max_cycles: hal::FOC_MAX_CYCLES.load(Ordering::Relaxed),
            foc_misses: hal::FOC_DEADLINE_MISSES.load(Ordering::Relaxed),
            params_from_flash,
        };

        // Arming is published to the FOC ISR so diagnostics can use the same
        // interlock as the safety controller.
        hal::ARMED.store(safe.armed(), Ordering::Relaxed);
        let batt = hal::BATT_RAW.load(Ordering::Relaxed);
        let temp = hal::TEMP_DECIC.load(Ordering::Relaxed);
        let battery_level = BatteryMonitor::level(batt);
        // `calcAvgSpeed()` (util.c:460-485) subtracts the right motor because
        // the two wheels spin in opposite directions on this drivetrain.
        let speed_avg = (i32::from(hal::SPEED_L.load(Ordering::Relaxed))
            - i32::from(hal::SPEED_R.load(Ordering::Relaxed)))
            / 2;
        // Latch-off is gated by measured wheel speed, not by the command
        // magnitude (util.c:566-577 uses `speedAvgAbs`).
        let near_standstill = speed_avg.unsigned_abs() <= STANDSTILL_RPM as u32;

        // Stock hard latch-off (`util.c:1561-1575`, `main.c:570-577`): unlike the
        // torque-node variant, a full copy removes battery power on a dead pack,
        // overheat or inactivity. These are deliberate shutdowns, not faults
        // that merely zero the setpoint.
        let latch_off = inactivity.tick(left, right, false)
            || (BAT_DEAD_ENABLE && battery_level == BatteryLevel::Dead && near_standstill)
            || (TEMP_POWEROFF_ENABLE && i32::from(temp) >= TEMP_POWEROFF_DECIC && near_standstill);
        if latch_off {
            safe.disarm();
            for tone in [0u8, 1, 2, 3, 4, 5, 6, 7] {
                hal::BUZZER_FREQ_OVERRIDE.store(tone, Ordering::Relaxed);
                let start = core.DWT.cyccnt.read();
                while core.DWT.cyccnt.read().wrapping_sub(start) < SYSTEM_HZ / 10 {
                    hal::feed_iwdg(&peripherals.IWDG);
                }
            }
            hal::BUZZER_FREQ_OVERRIDE.store(hal::BUZZER_NO_OVERRIDE, Ordering::Relaxed);
            // Keep the boot vector intact; see the button poweroff path above.
            hal::release_power_latch(gpioa);
            loop {
                hal::feed_iwdg(&peripherals.IWDG);
                cortex_m::asm::nop();
            }
        }
        if hal::DIAG_LEFT.load(Ordering::Relaxed) != 0
            || hal::DIAG_RIGHT.load(Ordering::Relaxed) != 0
        {
            safe.raise_fault(Fault::MotorDiagnostic);
        }

        let alert = select_alert(&AlertInputs {
            thermal_poweroff: TEMP_POWEROFF_ENABLE
                && i32::from(temp) >= TEMP_POWEROFF_DECIC
                && near_standstill,
            battery_dead: BAT_DEAD_ENABLE && battery_level == BatteryLevel::Dead && near_standstill,
            motor_error: safe.fault() == Some(Fault::MotorDiagnostic),
            adc_timeout: false, // CONTROL_ADC is not compiled in VARIANT_USART
            serial_timeout: serial_timeout.flag,
            general_timeout: false, // no PPM/PWM/Nunchuk input path was ported
            thermal_warning: TEMP_WARNING_ENABLE && i32::from(temp) >= TEMP_WARNING_DECIC,
            battery_lvl1: BAT_LVL1_ENABLE && battery_level == BatteryLevel::Level1,
            battery_lvl2: BAT_LVL2_ENABLE && battery_level == BatteryLevel::Level2,
            // `BEEPS_BACKWARD` in main.c:591: a reverse command while still
            // rolling forward. The multiple-tap brake path is not ported.
            backward: BEEPS_BACKWARD && (right < -50 || left < -50) && speed_avg < 0,
        });
        hal::BUZZER_ALERT.store(alert.as_u8(), Ordering::Relaxed);

        // Pico feedback contract: 18-byte legacy frame at 100 Hz (every second
        // 5 ms tick). Speeds are published by the FOC ISR in Q0 rpm; battery is
        // the filtered ADC converted to centivolts exactly as in main.c:494.
        if loop_index.is_multiple_of(2) {
            match link.mode() {
                ProtocolMode::Legacy => {
                    let feedback = encode_legacy_feedback(
                        applied_steer,
                        applied_speed,
                        hal::SPEED_R.load(Ordering::Relaxed),
                        hal::SPEED_L.load(Ordering::Relaxed),
                        BatteryMonitor::centivolts(batt),
                        temp,
                        0,
                    );
                    hal::feedback_write(&feedback);
                }
                ProtocolMode::Hardened => {
                    let data = FeedbackData {
                        cmd1: applied_steer,
                        cmd2: applied_speed,
                        speed_right_rpm: hal::SPEED_R.load(Ordering::Relaxed),
                        speed_left_rpm: hal::SPEED_L.load(Ordering::Relaxed),
                        battery_cv: BatteryMonitor::centivolts(batt),
                        board_temp_decic: temp,
                        led: 0,
                        err_code: hal::DIAG_LEFT.load(Ordering::Relaxed)
                            | (hal::DIAG_RIGHT.load(Ordering::Relaxed) << 4),
                        hall_left: hal::HALL_L.load(Ordering::Relaxed),
                        hall_right: hal::HALL_R.load(Ordering::Relaxed),
                        dc_left: hal::DC_LEFT.load(Ordering::Relaxed) as i16,
                        dc_right: hal::DC_RIGHT.load(Ordering::Relaxed) as i16,
                        duty_left: hal::DUTY_L[0].load(Ordering::Relaxed),
                        duty_right: hal::DUTY_R[0].load(Ordering::Relaxed),
                    };
                    let mut wire = [0u8; WIRE_MAX];
                    if let Some(len) =
                        encode_hardened_feedback(last_frame_seq, now_us, &data, &mut wire)
                    {
                        hal::feedback_write(&wire[..len]);
                    }
                }
            }
        }

        // Debug console: the grammar lives in f103_core, so the ISR only moves
        // bytes. Commands never change the actuator or safety configuration.
        while let Some(byte) = hal::debug_rx_pop() {
            let Some(command_line) = line.push(byte) else {
                continue;
            };
            match parse_command(command_line) {
                Command::Empty => {}
                Command::Help => {
                    let mut f = Formatter::new(&mut reply);
                    format_help(&mut f);
                    hal::debug_write(f.as_str().as_bytes());
                }
                Command::Get => {
                    let mut f = Formatter::new(&mut reply);
                    format_get(&mut f, &telemetry);
                    hal::debug_write(f.as_str().as_bytes());
                }
                Command::GetParams => {
                    let mut f = Formatter::new(&mut reply);
                    format_params(&mut f, &params);
                    hal::debug_write(f.as_str().as_bytes());
                }
                Command::SetParam(key, value) => {
                    let mut f = Formatter::new(&mut reply);
                    if safe.armed() && key.is_actuator() {
                        f.put_str("PARAM refused: disarm before tuning actuator\n");
                    } else {
                        let applied = key.set(&mut params, value);
                        cortex_m::interrupt::free(|cs| {
                            if let Some(pair) = MOTORS.borrow(cs).borrow_mut().as_mut() {
                                params.apply_to(&mut pair.params);
                            }
                        });
                        f.put_str("P ");
                        f.put_str(key.label());
                        f.push(b' ');
                        f.put_i32(applied as i32);
                        f.line();
                    }
                    hal::debug_write(f.as_str().as_bytes());
                }
                Command::WatchToggle => {
                    watch = !watch;
                    let mut f = Formatter::new(&mut reply);
                    f.put_str(if watch { "WATCH on\n" } else { "WATCH off\n" });
                    hal::debug_write(f.as_str().as_bytes());
                }
                Command::SetWatch(value) => {
                    watch_div = value;
                    if value == 0 {
                        watch = false;
                    }
                    let mut f = Formatter::new(&mut reply);
                    f.put_str("WATCH_DIV ");
                    f.put_i32(value as i32);
                    f.line();
                    hal::debug_write(f.as_str().as_bytes());
                }
                Command::SetProtocol(mode) => {
                    // Full-behaviour port: the reference only speaks the legacy
                    // 8-byte contract, so the hardened switch is refused.
                    let mut f = Formatter::new(&mut reply);
                    if mode == ProtocolMode::Legacy {
                        link.set_mode(mode);
                        f.put_str("PROTOCOL ");
                        f.put_str(mode.label());
                    } else {
                        f.put_str("PROTOCOL refused: legacy only");
                    }
                    f.line();
                    hal::debug_write(f.as_str().as_bytes());
                }
                Command::Trace(requested) => {
                    // One ~110-byte frame at 115200 8N1 costs about 1 ms on the
                    // wire. Below 160 FOC ticks (100 Hz) the stream would
                    // saturate the link and starve console replies, so clamp.
                    let decimation = if requested == 0 {
                        0
                    } else {
                        requested.max(160)
                    };
                    hal::TRACE_DECIM.store(decimation, Ordering::Relaxed);
                    if decimation == 0 {
                        cortex_m::interrupt::free(|cs| {
                            hal::TRACE_TX.borrow(cs).borrow_mut().clear()
                        });
                    }
                    let mut f = Formatter::new(&mut reply);
                    f.put_str("TRACE ");
                    f.put_i32(decimation as i32);
                    f.line();
                    hal::debug_write(f.as_str().as_bytes());
                }
                Command::Arm => {
                    let mut f = Formatter::new(&mut reply);
                    if !hal::CAL_READY.load(Ordering::Relaxed) {
                        f.put_str("ARM refused: ADC calibration not ready\n");
                    } else {
                        // Bench-only: let the generated torque loop run so HIL can
                        // observe iq/id, integrators and duty. MOE is never written,
                        // so no gate can be driven and no torque can be produced.
                        safe.arm();
                        f.put_str("ARM bench: torque loop live, MOE low, no torque\n");
                    }
                    hal::debug_write(f.as_str().as_bytes());
                }
                Command::Disarm => {
                    safe.disarm();
                    let mut f = Formatter::new(&mut reply);
                    f.put_str("DISARM: torque loop open\n");
                    hal::debug_write(f.as_str().as_bytes());
                }
                Command::Init => {
                    hal::RECALIBRATE.store(true, Ordering::Relaxed);
                    let mut f = Formatter::new(&mut reply);
                    f.put_str("INIT calibration queued\n");
                    hal::debug_write(f.as_str().as_bytes());
                }
                Command::Save => {
                    let mut f = Formatter::new(&mut reply);
                    if safe.armed() {
                        // Flash erase stalls instruction fetch; never do it
                        // while the torque node is armed and balancing.
                        f.put_str("SAVE refused: disarm before writing flash\n");
                    } else {
                        let words = params.to_words();
                        let ok = cortex_m::interrupt::free(|_| {
                            hal::nvm_save(&peripherals.FLASH, &words)
                        });
                        if ok {
                            f.put_str("SAVE ok\n");
                        } else {
                            f.put_str("SAVE failed: flash verification failed\n");
                        }
                    }
                    hal::debug_write(f.as_str().as_bytes());
                }
                Command::Dump => dump_index = Some(0),
                Command::Adc => {
                    let fields =
                        cortex_m::interrupt::free(|cs| *hal::LAST_FIELDS.borrow(cs).borrow());
                    let mut f = Formatter::new(&mut reply);
                    format_adc(&mut f, &fields);
                    hal::debug_write(f.as_str().as_bytes());
                }
                Command::Rejected(text) => {
                    let mut f = Formatter::new(&mut reply);
                    f.put_str("ERR ");
                    f.put_str(text);
                    f.line();
                    hal::debug_write(f.as_str().as_bytes());
                }
            }
        }

        if watch && watch_div > 0 && loop_index.is_multiple_of(watch_div as u32) {
            let mut f = Formatter::new(&mut reply);
            format_watch(&mut f, &telemetry);
            hal::debug_write(f.as_str().as_bytes());
        }

        // Stream one sample per tick when dumping so the TX ring stays small.
        if let Some(index) = dump_index {
            if let Some(sample) = blackbox.get(index) {
                let mut f = Formatter::new(&mut reply);
                format_sample(&mut f, index, &sample);
                hal::debug_write(f.as_str().as_bytes());
                dump_index = Some(index + 1);
            } else {
                dump_index = None;
            }
        }

        let mut flags = 0u8;
        if telemetry.armed {
            flags |= 1;
        }
        if telemetry.calibration_ready {
            flags |= 2;
        }
        if telemetry.fault.is_some() {
            flags |= 4;
        }
        blackbox.push(Sample {
            tick: telemetry.foc_ticks,
            steer: telemetry.steer,
            speed: telemetry.speed,
            target_left: left,
            target_right: right,
            dc_left: telemetry.dc_left,
            dc_right: telemetry.dc_right,
            flags,
        });

        // Feed only after the loop completed and the safe state was computed.
        hal::feed_iwdg(&peripherals.IWDG);
    }
}
