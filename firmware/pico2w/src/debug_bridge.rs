//! Transparent bridge between a PIO UART and the USB console.
//!
//! Bench-only: the STM32 debug serial on USART3 (right cable) is forwarded
//! through the existing Pico USB port, so no extra USB-UART adapter is needed.
//! Two bounded queues keep the USB console and the PIO tasks decoupled.

use embassy_executor::Spawner;
use embassy_rp::bind_interrupts;
use embassy_rp::peripherals::{PIN_12, PIN_13, PIO0};
use embassy_rp::pio::{InterruptHandler, Pio};
use embassy_rp::pio_programs::uart::{PioUartRx, PioUartRxProgram, PioUartTx, PioUartTxProgram};
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::channel::Channel;

bind_interrupts!(struct Irqs {
    PIO0_IRQ_0 => InterruptHandler<PIO0>;
});

const DEBUG_BAUD: u32 = 115_200;
const QUEUE_LEN: usize = 256;

static RX: Channel<CriticalSectionRawMutex, u8, QUEUE_LEN> = Channel::new();
static TX: Channel<CriticalSectionRawMutex, u8, QUEUE_LEN> = Channel::new();

/// Return one byte received from the debug UART, if any.
pub fn poll_rx() -> Option<u8> {
    RX.receiver().try_receive().ok()
}

/// Queue bytes to send to the debug UART; overflow is dropped.
pub fn push_tx(bytes: &[u8]) {
    let sender = TX.sender();
    for &byte in bytes {
        let _ = sender.try_send(byte);
    }
}

#[embassy_executor::task]
async fn rx_task(mut uart: PioUartRx<'static, PIO0, 0>) {
    let sender = RX.sender();
    loop {
        let byte = uart.read_u8().await;
        let _ = sender.try_send(byte);
    }
}

#[embassy_executor::task]
async fn tx_task(mut uart: PioUartTx<'static, PIO0, 1>) {
    let receiver = TX.receiver();
    loop {
        let byte = receiver.receive().await;
        uart.write_u8(byte).await;
    }
}

/// Start the PIO UART on GP12 (RX) and GP13 (TX) at 115200 8N1.
///
/// Wire STM32 USART3 TX (PB10) to GP12 and GP13 to USART3 RX (PB11), with a
/// shared ground. The bridge is inert until `dbg on` enables forwarding; the
/// tasks still drain the UART so boot messages are not lost.
pub fn init(periph: PIO0, rx_pin: PIN_12, tx_pin: PIN_13, spawner: &Spawner) {
    let pio = Pio::new(periph, Irqs);
    let embassy_rp::pio::Pio {
        mut common,
        sm0,
        sm1,
        ..
    } = pio;
    let rx_program = PioUartRxProgram::new(&mut common);
    let tx_program = PioUartTxProgram::new(&mut common);
    let rx = PioUartRx::new(DEBUG_BAUD, &mut common, sm0, rx_pin, &rx_program);
    let tx = PioUartTx::new(DEBUG_BAUD, &mut common, sm1, tx_pin, &tx_program);
    spawner.must_spawn(rx_task(rx));
    spawner.must_spawn(tx_task(tx));
}
