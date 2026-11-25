//! Minimal PAC bring-up for the donor mainboard.
//!
//! Only clocks, pins, timers and the two UARTs are configured here. PWM outputs
//! stay disabled (MOE low) and no motor can be driven until the FOC port passes
//! HIL validation. ADC/DMA, the independent watchdog and the debug console are
//! intentionally deferred to later stages of the staged plan.

use core::cell::{RefCell, UnsafeCell};
use core::sync::atomic::{
    AtomicBool, AtomicI16, AtomicI32, AtomicU16, AtomicU32, AtomicU8, Ordering,
};

use cortex_m::interrupt::Mutex;
use f103_core::config::{
    usart_brr, ADC_TRIGGER_OFFSET, APB1_HZ, PWM_ARR, PWM_DEAD_TIME, SYSTEM_HZ,
};
use f103_core::console::ByteRing;
use f103_core::sensing::ADC_FIELDS;
use stm32f1::stm32f103 as pac;

/// DMA-owned memory. The wrapper documents why sharing is sound: DMA writes
/// through a raw pointer and no Rust reference exists while a transfer runs.
pub struct DmaBuffer<T>(pub UnsafeCell<T>);
// SAFETY: access is restricted to raw-pointer reads/writes in this HAL module.
unsafe impl<T> Sync for DmaBuffer<T> {}

/// ADC1/ADC2 dual-mode DMA destination. Dual regular-simultaneous mode packs
/// both converters into one 32-bit word, so five ranks fill five words. A
/// four-byte-aligned `[u32; 5]` is required by the 32-bit DMA transfer.
pub static ADC_BUFFER: DmaBuffer<[u32; 5]> = DmaBuffer(UnsafeCell::new([0; 5]));
/// USART2 circular RX destination; the IDLE ISR publishes the write position.
pub static RX_BUFFER: DmaBuffer<[u8; 64]> = DmaBuffer(UnsafeCell::new([0; 64]));
pub static RX_READY: AtomicBool = AtomicBool::new(false);
/// DMA write position in `RX_BUFFER`. The main loop consumes only the span
/// between two positions, like `old_pos` in `util.c:1091`.
pub static RX_POS: AtomicU32 = AtomicU32::new(0);

/// FOC observability counters, sampled by the debug console on request.
pub static FOC_TICKS: AtomicU32 = AtomicU32::new(0);
pub static FOC_OVERCURRENT: AtomicU32 = AtomicU32::new(0);
pub static FOC_SKIPPED: AtomicU32 = AtomicU32::new(0);
/// DWT cycle counts of the full FOC ISR. Used to size the 62.5 us budget from
/// measurements instead of assumptions; they never affect the control path.
pub static FOC_LAST_CYCLES: AtomicU32 = AtomicU32::new(0);
pub static FOC_MAX_CYCLES: AtomicU32 = AtomicU32::new(0);
pub static FOC_DEADLINE_MISSES: AtomicU32 = AtomicU32::new(0);
/// One FOC period at 64 MHz. Crossing it means the ISR overran its deadline.
pub const FOC_CYCLE_BUDGET: u32 = SYSTEM_HZ / 16_000;

/// Actuator and sense snapshots published by the FOC ISR for the debug loop.
/// The ISR never waits for a consumer; the main loop reads the latest value.
pub static DUTY_L: [AtomicU16; 3] = [AtomicU16::new(0), AtomicU16::new(0), AtomicU16::new(0)];
pub static DUTY_R: [AtomicU16; 3] = [AtomicU16::new(0), AtomicU16::new(0), AtomicU16::new(0)];
pub static DC_LEFT: AtomicI32 = AtomicI32::new(0);
pub static DC_RIGHT: AtomicI32 = AtomicI32::new(0);
pub static CAL_READY: AtomicBool = AtomicBool::new(false);
pub static RECALIBRATE: AtomicBool = AtomicBool::new(false);

/// Monitor outputs published by the FOC ISR and consumed by the 5 ms loop.
pub static BATT_RAW: AtomicI16 = AtomicI16::new(0);
pub static TEMP_DECIC: AtomicI16 = AtomicI16::new(0);
pub static DIAG_LEFT: AtomicU8 = AtomicU8::new(0);
pub static DIAG_RIGHT: AtomicU8 = AtomicU8::new(0);
/// Raw hall codes in generated A<<2|B<<1|C order for feedback telemetry.
pub static HALL_L: AtomicU8 = AtomicU8::new(0);
pub static HALL_R: AtomicU8 = AtomicU8::new(0);
pub static ARMED: AtomicBool = AtomicBool::new(false);
/// Selected alert as `Alert::as_u8`; the FOC ISR drives the buzzer from it.
pub static BUZZER_ALERT: AtomicU8 = AtomicU8::new(0);
/// 0xFF = no override, otherwise a forced buzzer divider (0 = silence). The
/// power-on/off sweeps use this so an alert cannot interrupt their timing.
pub const BUZZER_NO_OVERRIDE: u8 = 0xFF;
pub static BUZZER_FREQ_OVERRIDE: AtomicU8 = AtomicU8::new(BUZZER_NO_OVERRIDE);

/// Packed ADC snapshot, replaced atomically (inside a critical section) once
/// per FOC tick so `$ADC` sees a consistent field set.
pub static LAST_FIELDS: Mutex<RefCell<[u16; ADC_FIELDS]>> =
    Mutex::new(RefCell::new([0; ADC_FIELDS]));

/// USART3 RX/TX rings. The ISR owns the producer side of RX and the consumer
/// side of TX; both are protected by the same critical section as the main loop.
const DEBUG_RING: usize = 256;
pub static DEBUG_RX: Mutex<RefCell<ByteRing<DEBUG_RING>>> =
    Mutex::new(RefCell::new(ByteRing::new()));
pub static DEBUG_TX: Mutex<RefCell<ByteRing<DEBUG_RING>>> =
    Mutex::new(RefCell::new(ByteRing::new()));
pub static DEBUG_DROPPED: AtomicU32 = AtomicU32::new(0);

/// USART2 feedback TX ring. The 100 Hz producer runs in the main loop and the
/// TXE ISR drains one byte at a time, so formatting never blocks control.
const FEEDBACK_RING: usize = 64;
pub static FEEDBACK_TX: Mutex<RefCell<ByteRing<FEEDBACK_RING>>> =
    Mutex::new(RefCell::new(ByteRing::new()));
pub static FEEDBACK_DROPPED: AtomicU32 = AtomicU32::new(0);

/// Binary FOC trace shares the USART3 TX line with the text console. When a
/// trace is active its ring has priority; a `$TRACE 0` stops the stream so the
/// console is usable again.
const TRACE_RING: usize = 128;
pub static TRACE_TX: Mutex<RefCell<ByteRing<TRACE_RING>>> =
    Mutex::new(RefCell::new(ByteRing::new()));
pub static TRACE_DROPPED: AtomicU32 = AtomicU32::new(0);
pub static TRACE_DECIM: AtomicU8 = AtomicU8::new(0);
/// Wheel speeds published by the FOC ISR in Q0 rpm for the feedback frame.
pub static SPEED_L: AtomicI16 = AtomicI16::new(0);
pub static SPEED_R: AtomicI16 = AtomicI16::new(0);

/// Snapshot of the packed ADC results as ten 16-bit fields in `adc_buf_t` order.
pub fn adc_fields() -> [u16; 10] {
    // SAFETY: DMA only writes this 'static, four-byte-aligned buffer. The
    // volatile read takes a consistent snapshot for the ISR to consume.
    let words = unsafe { core::ptr::read_volatile(ADC_BUFFER.0.get() as *const [u32; 5]) };
    let mut fields = [0u16; 10];
    for (index, word) in words.iter().enumerate() {
        fields[2 * index] = *word as u16;
        fields[2 * index + 1] = (*word >> 16) as u16;
    }
    fields
}

/// Critical-section implementation for a single Cortex-M core: masking IRQs is
/// sufficient because no other core shares these peripherals.
#[no_mangle]
unsafe extern "C" fn _critical_section_1_0_acquire() -> u32 {
    // SAFETY: PRIMASK is a read-only core register at this privilege level.
    let primask: u32;
    core::arch::asm!("mrs {}, primask", out(reg) primask, options(nomem, nostack));
    cortex_m::interrupt::disable();
    primask
}

#[no_mangle]
unsafe extern "C" fn _critical_section_1_0_release(primask: u32) {
    // SAFETY: restoring a previously read PRIMASK cannot enable an interrupt
    // that was masked before the critical section.
    if primask & 1 == 0 {
        cortex_m::interrupt::enable();
    }
}

#[derive(Clone, Copy)]
enum PinMode {
    Analog,
    FloatingInput,
    /// MODE=00, CNF=10 with ODR=0: input with pull-down on F103.
    AltPushPull,
    PushPullOutput,
}

fn set_mode(gpio: &pac::gpioa::RegisterBlock, pin: u8, mode: PinMode) {
    let index = pin & 7;
    macro_rules! apply_mode {
        ($w:ident) => {{
            match mode {
                PinMode::Analog => {
                    $w.mode(index).input();
                    $w.cnf(index).push_pull();
                }
                PinMode::FloatingInput => {
                    $w.mode(index).input();
                    // CNF=01 is the floating-input configuration on F103.
                    $w.cnf(index).open_drain();
                }
                PinMode::AltPushPull => {
                    $w.mode(index).output50();
                    $w.cnf(index).alt_push_pull();
                }
                PinMode::PushPullOutput => {
                    $w.mode(index).output50();
                    $w.cnf(index).push_pull();
                }
            }
            $w
        }};
    }
    if pin < 8 {
        gpio.crl().modify(|_, w| apply_mode!(w));
    } else {
        // CRH has the same field layout as CRL; both alias the GPIOA block.
        gpio.crh().modify(|_, w| apply_mode!(w));
    }
}

/// HSI/2 x PLL16 = 64 MHz, APB1 /2, APB2 /1, matching the flashed reference.
pub fn init_clocks(rcc: &pac::RCC, flash: &pac::FLASH) {
    // 64 MHz needs two flash wait states. Without this the core faults on
    // instruction fetch as soon as the PLL becomes the system clock.
    flash
        .acr()
        .modify(|_, w| w.latency().ws2().prftbe().set_bit());
    rcc.cr().modify(|_, w| w.hsion().on());
    while !rcc.cr().read().hsirdy().is_ready() {}

    rcc.cfgr().modify(|_, w| {
        w.sw().hsi();
        w.hpre().div1();
        w.ppre1().div2();
        w.ppre2().div1();
        w.adcpre().div4();
        w.pllsrc().hsi_div2();
        w.pllmul().mul16();
        w
    });
    rcc.cr().modify(|_, w| w.pllon().on());
    while !rcc.cr().read().pllrdy().is_ready() {}

    rcc.cfgr().modify(|_, w| w.sw().pll());
    while !rcc.cfgr().read().sws().is_pll() {}
}

pub fn init_power_and_gpio(
    rcc: &pac::RCC,
    gpioa: &pac::GPIOA,
    gpiob: &pac::GPIOB,
    gpioc: &pac::GPIOC,
) {
    rcc.apb2enr().modify(|_, w| {
        w.iopaen().set_bit();
        w.iopben().set_bit();
        w.iopcen().set_bit();
        w.afioen().set_bit();
        w
    });

    // TIM8 left high/low sides: PC6/PC7/PC8 and PA7/PB0/PB1.
    set_mode(gpioc, 6, PinMode::AltPushPull);
    set_mode(gpioc, 7, PinMode::AltPushPull);
    set_mode(gpioc, 8, PinMode::AltPushPull);
    set_mode(gpioa, 7, PinMode::AltPushPull);
    set_mode(gpiob, 0, PinMode::AltPushPull);
    set_mode(gpiob, 1, PinMode::AltPushPull);

    // TIM1 right high/low sides: PA8/PA9/PA10 and PB13/PB14/PB15.
    set_mode(gpioa, 8, PinMode::AltPushPull);
    set_mode(gpioa, 9, PinMode::AltPushPull);
    set_mode(gpioa, 10, PinMode::AltPushPull);
    set_mode(gpiob, 13, PinMode::AltPushPull);
    set_mode(gpiob, 14, PinMode::AltPushPull);
    set_mode(gpiob, 15, PinMode::AltPushPull);

    // Hall inputs: PB5/PB6/PB7 and PC10/PC11/PC12.
    for pin in [5, 6, 7] {
        set_mode(gpiob, pin, PinMode::FloatingInput);
    }
    for pin in [10, 11, 12] {
        set_mode(gpioc, pin, PinMode::FloatingInput);
    }

    // USART2 PA2 TX / PA3 RX; USART3 PB10 TX / PB11 RX.
    set_mode(gpioa, 2, PinMode::AltPushPull);
    set_mode(gpioa, 3, PinMode::FloatingInput);
    set_mode(gpiob, 10, PinMode::AltPushPull);
    set_mode(gpiob, 11, PinMode::FloatingInput);

    // ADC current shunts and pack divider.
    for pin in [0, 1, 2, 3, 4, 5] {
        set_mode(gpioc, pin, PinMode::Analog);
    }
    set_mode(gpioa, 0, PinMode::Analog);

    // The power button is active high. The reference uses GPIO_NOPULL here
    // (setup.c:401-404): the sense line is weak, and an internal pull-down can
    // hold it below the logic threshold even while the button is pressed.
    set_mode(gpioa, 1, PinMode::FloatingInput);
    set_mode(gpiob, 2, PinMode::PushPullOutput);
    set_mode(gpioa, 4, PinMode::PushPullOutput);
    set_mode(gpioa, 5, PinMode::PushPullOutput);
    // The reference waits 500 ms before latching and lights the LED only after
    // the startup melody. Both are driven from main, not from GPIO setup.
}

/// Latch power on (PA5). Called after the reference 500 ms startup delay.
pub fn latch_power(gpioa: &pac::gpioa::RegisterBlock) {
    gpioa.odr().modify(|_, w| w.odr5().set_bit());
}

/// Board-ready LED (PB2), lit after the startup melody like `main.c:222`.
pub fn led_on(gpiob: &pac::gpiob::RegisterBlock) {
    gpiob.odr().modify(|_, w| w.odr2().set_bit());
}

/// The power button is active high on the v2 board (`defines.h:141-142`).
pub fn power_button_pressed(gpioa: &pac::gpioa::RegisterBlock) -> bool {
    gpioa.idr().read().idr1().bit()
}

/// Releases the firmware-held latch (`util.c:1573`). After this the board only
/// stays alive while the button is physically held or the probe back-powers it.
pub fn release_power_latch(gpioa: &pac::gpioa::RegisterBlock) {
    gpioa.odr().modify(|_, w| w.odr5().clear_bit());
}

fn init_timer(tim: &pac::tim1::RegisterBlock) {
    // SAFETY: ARR and PSC are 16-bit counter values; the constants are in range.
    tim.arr().write(|w| unsafe { w.arr().bits(PWM_ARR) });
    tim.psc().write(|w| unsafe { w.psc().bits(0) });
    tim.cr1().modify(|_, w| {
        w.cms().center_aligned1();
        w.arpe().clear_bit();
        w
    });
    tim.bdtr().modify(|_, w| {
        // SAFETY: DTG is an 8-bit dead-time value, any value is representable.
        unsafe { w.dtg().bits(PWM_DEAD_TIME) };
        w.ossr().idle_level();
        w.ossi().idle_level();
        // MOE stays low until self-test passes.
        w.moe().disabled_idle();
        w
    });
    for channel in 0..3 {
        // SAFETY: zero is a valid duty value for a 16-bit compare register.
        tim.ccr(channel as usize)
            .write(|w| unsafe { w.ccr().bits(0) });
    }
    tim.ccer().modify(|_, w| {
        w.cc1e().set_bit();
        w.cc1ne().set_bit();
        w.cc2e().set_bit();
        w.cc2ne().set_bit();
        w.cc3e().set_bit();
        w.cc3ne().set_bit();
        // Main outputs active high, complementary outputs active low, as in
        // the pinned C build. This is part of the gate-drive polarity contract.
        w.cc1p().clear_bit();
        w.cc1np().set_bit();
        w.cc2p().clear_bit();
        w.cc2np().set_bit();
        w.cc3p().clear_bit();
        w.cc3np().set_bit();
        w
    });
    tim.ccmr1_output().modify(|_, w| {
        w.oc1m().pwm_mode1();
        w.oc2m().pwm_mode1();
        w
    });
    tim.ccmr2_output().modify(|_, w| w.oc3m().pwm_mode1());
}

pub fn init_pwm(rcc: &pac::RCC, tim1: &pac::TIM1, tim8: &pac::TIM8) {
    rcc.apb2enr().modify(|_, w| {
        w.tim1en().set_bit();
        w.tim8en().set_bit();
        w
    });
    init_timer(tim1);
    init_timer(tim8);
    // Timers are configured but not started. start_pwm() enables them only
    // after the ADC and its trigger are ready. MOE is never set here.
}

/// Start the sampling chain: TIM8 update drives ADC1 through the AFIO remap,
/// TIM1 is a gated slave of TIM8 with a conversion-time offset, and the DMA
/// transfer-complete IRQ becomes the 16 kHz FOC tick.
pub fn start_pwm(afio: &pac::AFIO, tim1: &pac::TIM1, tim8: &pac::TIM8) {
    afio.mapr().modify(|_, w| w.adc1_etrgreg_remap().set_bit());

    // TIM1 (right) is the master and announces its enable on TRGO; TIM8 (left)
    // is a gated slave of that signal and starts preloaded by one conversion
    // time. TIM8's own update event is the ADC trigger.
    tim1.cr2().modify(|_, w| w.mms().enable());
    tim8.cr2().modify(|_, w| w.mms().update());
    tim8.smcr().modify(|_, w| {
        w.ts().itr0();
        w.sms().gated_mode();
        w
    });
    // SAFETY: the offset is well inside the 16-bit counter range.
    tim8.cnt()
        .write(|w| unsafe { w.cnt().bits(ADC_TRIGGER_OFFSET) });

    // Enabling the counters cannot drive a gate while MOE stays low.
    tim8.cr1().modify(|_, w| w.cen().set_bit());
    tim1.cr1().modify(|_, w| w.cen().set_bit());
}

fn init_uart(usart: &pac::usart1::RegisterBlock, baud: u32) {
    // USART2 and USART3 hang off APB1 (PCLK1 = 32 MHz), not SYSCLK. Using
    // SYSTEM_HZ here would program 57600 on a link pinned at 115200.
    let brr = usart_brr(APB1_HZ, baud);
    usart.brr().write(|w| {
        // SAFETY: mantissa and fraction are within their 12-bit/4-bit fields.
        unsafe { w.div_mantissa().bits((brr >> 4) as u16) };
        unsafe { w.div_fraction().bits((brr & 0xF) as u8) };
        w
    });
    usart.cr1().modify(|_, w| {
        w.te().set_bit();
        w.re().set_bit();
        w.ue().set_bit();
        w
    });
}

pub fn init_uarts(rcc: &pac::RCC, usart2: &pac::USART2, usart3: &pac::USART3) {
    rcc.apb1enr().modify(|_, w| {
        w.usart2en().set_bit();
        w.usart3en().set_bit();
        w
    });
    // USART2 is the control link; USART3 is the dedicated debug console.
    init_uart(usart2, 115_200);
    init_uart(usart3, 115_200);
}

/// ADC1+ADC2 dual regular-simultaneous, triggered by TIM8 TRGO, DMA1 channel 1.
/// Rank order matches `adc_buf_t` in the pinned C build.
pub fn init_adc_dma(rcc: &pac::RCC, dma1: &pac::DMA1, adc1: &pac::ADC1, adc2: &pac::ADC2) {
    rcc.ahbenr().modify(|_, w| w.dma1en().set_bit());
    rcc.apb2enr().modify(|_, w| {
        w.adc1en().set_bit();
        w.adc2en().set_bit();
        w
    });

    // SAFETY: the static is an UnsafeCell owned by DMA for the lifetime of the
    // firmware; no Rust reference to it is held while the transfer runs.
    let adc_ptr = ADC_BUFFER.0.get() as u32;
    let dr_ptr = pac::ADC1::ptr() as u32 + 0x4c;
    let channel = dma1.ch(0);
    channel.par().write(|w| unsafe { w.bits(dr_ptr) });
    channel.mar().write(|w| unsafe { w.bits(adc_ptr) });
    channel.ndtr().write(|w| unsafe { w.bits(5) });
    channel.cr().modify(|_, w| {
        w.dir().from_peripheral();
        w.circ().enabled();
        w.minc().enabled();
        // Dual regular-simultaneous mode packs ADC1 and ADC2 into one word.
        w.psize().bits32();
        w.msize().bits32();
        // The transfer-complete IRQ is the 16 kHz FOC tick, as in the C build.
        w.tcie().enabled();
        w.en().enabled();
        w
    });

    adc1.cr1().modify(|_, w| {
        w.scan().enabled();
        w.dualmod().regular();
        w
    });
    adc1.cr2().modify(|_, w| {
        w.cont().single();
        w.dma().enabled();
        w.exttrig().enabled();
        // EXTSEL value 6 is TIM8_TRGO on high-density F103 devices.
        w.extsel().exti11();
        w.tsvrefe().enabled();
        w
    });
    // SAFETY: all channel numbers fit the 5-bit SQ fields.
    adc1.sqr1().modify(|_, w| unsafe { w.l().bits(4) });
    adc1.sqr3().modify(|_, w| unsafe {
        w.sq1().bits(11);
        w.sq2().bits(0);
        w.sq3().bits(14);
        w.sq4().bits(12);
        w.sq5().bits(16);
        w
    });
    adc1.smpr1().modify(|_, w| unsafe {
        w.smp11().cycles1_5();
        w.smp14().bits(1);
        w.smp12().bits(1);
        w.smp16().cycles239_5();
        w
    });
    adc1.smpr2().modify(|_, w| unsafe {
        w.smp0().bits(1);
        w
    });
    adc1.cr2().modify(|_, w| w.adon().set_bit());

    adc2.cr1().modify(|_, w| w.scan().enabled());
    adc2.cr2().modify(|_, w| {
        w.cont().single();
        // The pinned build sets this on both converters in dual mode even
        // though only ADC1's DR is the DMA source.
        w.dma().enabled();
        w.adon().set_bit();
        w
    });
    adc2.sqr1().modify(|_, w| unsafe { w.l().bits(4) });
    adc2.sqr3().modify(|_, w| unsafe {
        w.sq1().bits(10);
        w.sq2().bits(13);
        w.sq3().bits(15);
        w.sq4().bits(2);
        w.sq5().bits(3);
        w
    });
    // CH10 keeps the 1.5-cycle reset value; the phase inputs use 7.5 cycles.
    adc2.smpr1().modify(|_, w| {
        w.smp13().cycles7_5();
        w.smp15().cycles7_5();
        w
    });
    adc2.smpr2().modify(|_, w| {
        w.smp2().cycles7_5();
        w.smp3().cycles7_5();
        w
    });
}

/// Circular RX on USART2 with IDLE detection. The ISR publishes the length;
/// the main loop parses bytes without ever blocking the control path.
pub fn init_usart2_rx_dma(dma1: &pac::DMA1, usart2: &pac::USART2) {
    // SAFETY: RX_BUFFER is an UnsafeCell dedicated to this DMA channel.
    let rx_ptr = RX_BUFFER.0.get() as u32;
    let dr_ptr = pac::USART2::ptr() as u32 + 0x04;
    let channel = dma1.ch(5);
    channel.par().write(|w| unsafe { w.bits(dr_ptr) });
    channel.mar().write(|w| unsafe { w.bits(rx_ptr) });
    channel.ndtr().write(|w| unsafe { w.bits(64) });
    channel.cr().modify(|_, w| {
        w.dir().from_peripheral();
        w.circ().enabled();
        w.minc().enabled();
        w.psize().bits8();
        w.msize().bits8();
        w.en().enabled();
        w
    });
    usart2.cr3().modify(|_, w| w.dmar().set_bit());
    usart2.cr1().modify(|_, w| w.idleie().set_bit());
    RX_POS.store(0, Ordering::Relaxed);
}

/// Start of the two flash pages reserved for parameters in `memory.x`. The
/// top 4 KB is excluded from the image, so erasing it cannot damage code.
pub const NVM_PAGE0: u32 = 0x0803_F000;
pub const NVM_PAGE1: u32 = 0x0803_F800;
const NVM_KEY1: u32 = 0x4567_0123;
const NVM_KEY2: u32 = 0xCDEF_89AB;

fn nvm_wait_busy(flash: &pac::FLASH) -> bool {
    // A bounded spin keeps a wedged flash controller from hanging the loop.
    for _ in 0..2_000_000 {
        cortex_m::asm::nop();
        if !flash.sr().read().bsy().bit_is_set() {
            return true;
        }
    }
    false
}

fn nvm_unlock(flash: &pac::FLASH) {
    flash.keyr().write(|w| unsafe { w.bits(NVM_KEY1) });
    flash.keyr().write(|w| unsafe { w.bits(NVM_KEY2) });
}

fn nvm_lock(flash: &pac::FLASH) {
    flash.cr().modify(|_, w| w.lock().set_bit());
}

fn nvm_erase_page(flash: &pac::FLASH) -> bool {
    flash.cr().modify(|_, w| {
        w.per().set_bit();
        w
    });
    flash.cr().modify(|_, w| {
        w.strt().set_bit();
        w
    });
    let done = nvm_wait_busy(flash);
    flash.cr().modify(|_, w| {
        w.per().clear_bit();
        w
    });
    done
}

/// Read the first valid parameter record. A foreign key, version or CRC is
/// treated as an empty store, exactly as `EE_Init` does in the reference.
pub fn nvm_restore(words: &mut [u16; f103_core::parameters::PARAM_WORDS]) -> bool {
    for base in [NVM_PAGE0, NVM_PAGE1] {
        for (index, word) in words.iter_mut().enumerate() {
            let address = (base + (index * 2) as u32) as *const u16;
            // SAFETY: the pages are outside the linked image and 16-bit aligned.
            *word = unsafe { core::ptr::read_volatile(address) };
        }
        if f103_core::parameters::Parameters::is_valid_record(words) {
            return true;
        }
    }
    false
}

/// Persist one record. Erase and program happen with interrupts masked by the
/// caller; the FOC ISR must never try to fetch instructions from flash while
/// the controller is busy. Returns true only after a read-back check.
pub fn nvm_save(flash: &pac::FLASH, words: &[u16; f103_core::parameters::PARAM_WORDS]) -> bool {
    nvm_unlock(flash);
    let mut written = false;
    for base in [NVM_PAGE0, NVM_PAGE1] {
        flash.ar().write(|w| unsafe { w.bits(base) });
        if !nvm_erase_page(flash) {
            continue;
        }
        flash.cr().modify(|_, w| {
            w.pg().set_bit();
            w
        });
        for (index, word) in words.iter().enumerate() {
            let address = (base + (index * 2) as u32) as *mut u16;
            // SAFETY: the page is reserved, aligned, and erased before writing.
            unsafe { core::ptr::write_volatile(address, *word) };
            if !nvm_wait_busy(flash) {
                break;
            }
        }
        flash.cr().modify(|_, w| {
            w.pg().clear_bit();
            w
        });
        // EOP is write-one-to-clear on the F103.
        flash.sr().modify(|_, w| {
            w.eop().set_bit();
            w
        });
        written = (0..words.len()).all(|index| {
            let address = (base + (index * 2) as u32) as *const u16;
            // SAFETY: reserved, aligned flash that was just programmed.
            (unsafe { core::ptr::read_volatile(address) }) == words[index]
        });
        if written {
            break;
        }
    }
    nvm_lock(flash);
    written
}

/// Enable the DWT cycle counter before any interrupt can sample it. The
/// addresses are fixed Cortex-M3 trace registers, so no PAC dependence.
pub fn init_dwt() {
    const DEMCR: *mut u32 = 0xE000_EDFC as *mut u32;
    const DWT_CONTROL: *mut u32 = 0xE000_1000 as *mut u32;
    const DWT_CYCCNT: *mut u32 = 0xE000_1004 as *mut u32;
    unsafe {
        // SAFETY: these are write-only-at-this-privilege trace registers.
        core::ptr::write_volatile(DEMCR, core::ptr::read_volatile(DEMCR) | (1 << 24));
        core::ptr::write_volatile(DWT_CYCCNT, 0);
        core::ptr::write_volatile(DWT_CONTROL, core::ptr::read_volatile(DWT_CONTROL) | 1);
    }
}

/// Read the free-running cycle counter. Wraps at 2^32 but ISR deltas are small.
pub fn cycles() -> u32 {
    // SAFETY: initialised by `init_dwt` before interrupts are unmasked.
    unsafe { core::ptr::read_volatile(0xE000_1004 as *const u32) }
}

/// Independent watchdog: LSI/64 with a 625-count reload gives roughly 1 s.
pub fn init_iwdg(iwdg: &pac::IWDG) {
    iwdg.kr().write(|w| w.key().unlock());
    // SAFETY: prescaler/reload values are within their 3-bit/12-bit fields.
    iwdg.pr().write(|w| unsafe { w.bits(4) });
    iwdg.rlr().write(|w| unsafe { w.bits(625) });
    iwdg.kr().write(|w| w.key().feed());
    iwdg.kr().write(|w| w.key().start());
}

/// Main output enable for both three-phase bridges (TIM1 right, TIM8 left).
/// This is the single point that can energize the gate drivers; the default
/// build never calls it with `true`.
#[cfg(feature = "moe")]
pub fn set_output_enable(
    tim1: &pac::tim1::RegisterBlock,
    tim8: &pac::tim8::RegisterBlock,
    on: bool,
) {
    tim1.bdtr().modify(|_, w| {
        if on {
            w.moe().set_bit();
        } else {
            w.moe().clear_bit();
        }
        w
    });
    tim8.bdtr().modify(|_, w| {
        if on {
            w.moe().set_bit();
        } else {
            w.moe().clear_bit();
        }
        w
    });
}

/// Drive the active-high buzzer gate on PA4. Called from the FOC ISR so the
/// tone timing does not depend on the 5 ms loop.
pub fn set_buzzer(gpioa: &pac::gpioa::RegisterBlock, level: bool) {
    gpioa.odr().modify(|_, w| {
        if level {
            w.odr4().set_bit();
        } else {
            w.odr4().clear_bit();
        }
        w
    });
}

/// Feed only after the main-loop health checks have passed.
pub fn feed_iwdg(iwdg: &pac::IWDG) {
    iwdg.kr().write(|w| w.key().feed());
}

/// Debug RX producer, called from the USART3 ISR.
pub fn debug_rx_push(byte: u8) {
    cortex_m::interrupt::free(|cs| {
        if !DEBUG_RX.borrow(cs).borrow_mut().push(byte) {
            DEBUG_DROPPED.fetch_add(1, Ordering::Relaxed);
        }
    });
}

/// Debug RX consumer, called from the main loop.
pub fn debug_rx_pop() -> Option<u8> {
    cortex_m::interrupt::free(|cs| DEBUG_RX.borrow(cs).borrow_mut().pop())
}

/// Queue an ASCII reply without blocking. The TXE ISR drains the ring, so a
/// large `$DUMP` cannot stall the 5 ms control loop.
pub fn debug_write(bytes: &[u8]) {
    cortex_m::interrupt::free(|cs| {
        {
            let mut tx = DEBUG_TX.borrow(cs).borrow_mut();
            for byte in bytes {
                if !tx.push(*byte) {
                    DEBUG_DROPPED.fetch_add(1, Ordering::Relaxed);
                }
            }
        }
        // SAFETY: USART3 is owned by this firmware; the interrupt-free section
        // serialises the flag update with the ISR's empty-ring check.
        let usart = unsafe { &*pac::USART3::ptr() };
        usart.cr1().modify(|_, w| w.txeie().set_bit());
    });
}

/// Called from the USART3 ISR: emits one queued byte, or disables TXE when the
/// ring runs dry to avoid an interrupt storm.
pub fn debug_tx_drain(usart: &pac::usart1::RegisterBlock) {
    cortex_m::interrupt::free(|cs| {
        // A binary trace frame must never be interleaved with ASCII replies.
        let byte = if TRACE_DECIM.load(Ordering::Relaxed) != 0 {
            TRACE_TX.borrow(cs).borrow_mut().pop()
        } else {
            DEBUG_TX.borrow(cs).borrow_mut().pop()
        };
        match byte {
            Some(byte) => usart.dr().write(|w| unsafe { w.dr().bits(byte as u16) }),
            None => usart.cr1().modify(|_, w| w.txeie().clear_bit()),
        }
    });
}

/// Queue one complete binary trace frame. The ISR only copies bytes into the
/// ring; encoding and formatting stay in the FOC tick where the state exists.
pub fn trace_write(bytes: &[u8]) {
    cortex_m::interrupt::free(|cs| {
        {
            let mut tx = TRACE_TX.borrow(cs).borrow_mut();
            for byte in bytes {
                if !tx.push(*byte) {
                    TRACE_DROPPED.fetch_add(1, Ordering::Relaxed);
                }
            }
        }
        let usart = unsafe { &*pac::USART3::ptr() };
        usart.cr1().modify(|_, w| w.txeie().set_bit());
    });
}

/// Queue one complete feedback frame for the control link. The TXE ISR owns the
/// byte-at-a-time drain, so an 18-byte frame cannot stall the 5 ms loop.
pub fn feedback_write(bytes: &[u8]) {
    cortex_m::interrupt::free(|cs| {
        {
            let mut tx = FEEDBACK_TX.borrow(cs).borrow_mut();
            for byte in bytes {
                if !tx.push(*byte) {
                    FEEDBACK_DROPPED.fetch_add(1, Ordering::Relaxed);
                }
            }
        }
        // SAFETY: USART2 is owned by this firmware; the interrupt-free section
        // serialises the flag update with the ISR's empty-ring check.
        let usart = unsafe { &*pac::USART2::ptr() };
        usart.cr1().modify(|_, w| w.txeie().set_bit());
    });
}

/// Called from the USART2 ISR: drains the feedback ring one byte at a time.
pub fn feedback_tx_drain(usart: &pac::usart1::RegisterBlock) {
    cortex_m::interrupt::free(|cs| {
        let byte = FEEDBACK_TX.borrow(cs).borrow_mut().pop();
        match byte {
            Some(byte) => usart.dr().write(|w| unsafe { w.dr().bits(byte as u16) }),
            None => usart.cr1().modify(|_, w| w.txeie().clear_bit()),
        }
    });
}
