//! Bench-only probe image: 64 MHz PLL, PB2 (LED) and PA5 (power latch) high.
//! No interrupts, no FOC, no console. Used to isolate board/firmware faults.

#![no_std]
#![no_main]

use cortex_m_rt::entry;
use panic_halt as _;
use stm32f1::stm32f103 as pac;

#[no_mangle]
unsafe extern "C" fn _critical_section_1_0_acquire() -> u32 {
    let primask: u32;
    core::arch::asm!("mrs {}, primask", out(reg) primask, options(nomem, nostack));
    cortex_m::interrupt::disable();
    primask
}

#[no_mangle]
unsafe extern "C" fn _critical_section_1_0_release(primask: u32) {
    if primask & 1 == 0 {
        cortex_m::interrupt::enable();
    }
}

#[entry]
fn main() -> ! {
    let p = pac::Peripherals::take().unwrap();

    p.FLASH
        .acr()
        .modify(|_, w| w.latency().ws2().prftbe().set_bit());

    p.RCC.cr().modify(|_, w| w.hsion().on());
    while !p.RCC.cr().read().hsirdy().is_ready() {}
    p.RCC.cfgr().modify(|_, w| {
        w.sw().hsi();
        w.hpre().div1();
        w.ppre1().div2();
        w.ppre2().div1();
        w.adcpre().div4();
        w.pllsrc().hsi_div2();
        w.pllmul().mul16();
        w
    });
    p.RCC.cr().modify(|_, w| w.pllon().on());
    while !p.RCC.cr().read().pllrdy().is_ready() {}
    p.RCC.cfgr().modify(|_, w| w.sw().pll());
    while !p.RCC.cfgr().read().sws().is_pll() {}

    p.RCC
        .apb2enr()
        .modify(|_, w| w.iopaen().set_bit().iopben().set_bit());
    p.GPIOB
        .crl()
        .modify(|_, w| w.mode(2).output50().cnf(2).push_pull());
    p.GPIOA
        .crl()
        .modify(|_, w| w.mode(5).output50().cnf(5).push_pull());
    p.GPIOB.odr().modify(|_, w| w.odr2().set_bit());
    p.GPIOA.odr().modify(|_, w| w.odr5().set_bit());

    loop {
        cortex_m::asm::nop();
    }
}
