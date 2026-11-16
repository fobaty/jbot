#![no_std]
#![no_main]

use defmt::info;
use embassy_rp::init;
use embassy_time::{Duration, Ticker};

#[allow(unused)]
fn main() {
    let config = embassy_rp::config::Config::default();
    init(config);

    info!("Pico 2 W bridge firmware starting");

    let mut ticker = Ticker::every(Duration::from_millis(5)); // 200 Hz

    loop {
        // Balance loop: read IMU, compute control, send torque to mainboard
        // TODO: implement full balance controller
        ticker.next();
    }
}

#[panic_handler]
fn panic(_info: &core::panic::PanicInfo) -> ! {
    loop {}
}
