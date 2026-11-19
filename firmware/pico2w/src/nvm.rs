//! Flash-backed settings storage for the bridge firmware.
//!
//! The record lives in the last erase block of the 4 MiB QSPI flash. The
//! linker is told to stop one block short (`memory.x`), so the sector can
//! never overlap the application image; nothing else in the firmware touches
//! this address.
//!
//! Writing a record erases and reprograms a whole block, which parks the XIP
//! cache and stalls every task for tens of milliseconds. The console therefore
//! saves only while the balance law is disarmed: no torque is being commanded,
//! the mainboard keeps running on its own serial timeout, and a missed control
//! tick cannot move the wheels.

use embassy_rp::flash::{Blocking, Flash};
use embassy_rp::peripherals::FLASH;
use embedded_storage::nor_flash::ReadNorFlash;

use jbot_pico_core::nvm::{Record, ERASE_BLOCK, RECORD_LEN};

/// Size of the QSPI flash on the Pico 2 W, bytes (4 MiB).
pub const FLASH_SIZE: usize = 4 * 1024 * 1024;
/// Offset of the settings block from the start of the flash, bytes.
pub const SLOT_OFFSET: u32 = (FLASH_SIZE - ERASE_BLOCK) as u32;

/// Blocking handle on the whole flash.
pub type Store = Flash<'static, FLASH, Blocking, FLASH_SIZE>;

/// Read the stored record, or [`None`] when the block is blank or damaged.
pub fn load(store: &mut Store) -> Option<Record> {
    let mut buffer = [0u8; RECORD_LEN];
    store.read(SLOT_OFFSET, &mut buffer).ok()?;
    Record::decode(&buffer)
}

/// Rewrite the settings block with `record`. Erases the block first, so a
/// power loss mid-write leaves a blank sector rather than a half record: the
/// embedded CRC would reject it anyway, and the firmware then falls back to
/// the compiled defaults.
pub fn save(store: &mut Store, record: Record) -> Result<(), ()> {
    use embedded_storage::nor_flash::NorFlash;

    let image = record.encode();
    store
        .erase(SLOT_OFFSET, SLOT_OFFSET + ERASE_BLOCK as u32)
        .map_err(|_| ())?;
    store.write(SLOT_OFFSET, &image).map_err(|_| ())
}

/// Erase the settings block, discarding any stored record.
pub fn forget(store: &mut Store) -> Result<(), ()> {
    use embedded_storage::nor_flash::NorFlash;

    store
        .erase(SLOT_OFFSET, SLOT_OFFSET + ERASE_BLOCK as u32)
        .map_err(|_| ())
}
