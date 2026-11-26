# Pico 2 W firmware: known-good vs known-bad (2026-11-25)

## Purpose

Record the single Pico image that is proven to boot on the bench, so the same
failed experiments are not repeated. Do not replace the currently flashed Pico
firmware until the regression described below is identified and fixed.

## Known-good image

| Property | Value |
|---|---|
| Source commit | `8f95b46` (`feat: calibrate hub breakaway and add delay predictor`), 2026-11-24 |
| Why this commit | Last Pico firmware commit before `221dc37`, which introduced the Rust torque node for the STM32F103 |
| Build tree layout | Flat: `firmware/pico2w/src/` plus sibling `firmware/pico_core/` |
| Source ELF SHA-256 | `4cb54e0d87ec68a60b0c9cf10dc2cf45d20dc7ddc4e41cbe8d0df5e9ebc63ca1` |
| Flashed UF2 SHA-256 | `659d91554b3e1aa8b8177f568721bd809bd611502f000951bdbb90479fdefcb6` |
| UF2 size | 276 KB (converted from ELF with `picotool uf2 convert`, so boot2 and image-def are included) |

Evidence on the bench, 2026-11-25: after `picotool load -u -v -x` the CDC
port `/dev/cu.usbmodemJBOT_PICO_011` appeared about 1.2 s after reboot;
`ping` returned `ok ping`; `stat` returned a full record with `loop=200`,
`pscale=1.00`, `slew=3000`. The same board and cable had produced no CDC with
every image listed below.

## Known-bad images (all failed identically)

| Image | SHA-256 | Origin |
|---|---|---|
| Current working-tree build | `a17a8314…` | `firmware/pico2w/fw` with the precision patches |
| Clean `git HEAD` build (`ed8bb1e`) | `f62f535c…` | unmodified committed tree |
| `f103-pico-clean` build from 13:51 | `95ed270f…` | HEAD plus one debug-bridge newline fix |
| `pico-221.uf2` | `122f48aa…` | commit `221dc37`, first Rust-STM32 commit |
| UF2 converted from the current build | `617ca71a…` | current build plus boot2/image-def |

Failure signature: `picotool load -u -v -x` reports `OK` and reboots into the
application, but no `/dev/cu.usbmodem*` appears, the device is not in BOOTSEL,
and no RP-series device is visible on USB. Holding BOOTSEL still makes the ROM
mass-storage device appear, so cable and power are fine. A full `picotool
erase -a` followed by a UF2 write did not change the result.

Because both ELF-only and UF2-with-boot2 flashes fail the same way, and the
known-good image boots on the same board, the regression is in the Pico source
between `8f95b46` and `221dc37` or in the later split into `firmware/pico2w/core`.

## Reproduce the known-good build

```sh
cd /Users/fobaty/dev/jbot
TMP=/private/var/folders/w5/h_z4zx4j35n45c5p7mj1hd9m0000gn/T/opencode/pico-good
rm -rf "$TMP" && mkdir -p "$TMP"
git worktree add --detach "$TMP/tree" 8f95b46
cd "$TMP/tree/firmware/pico2w"
cargo build --release
picotool uf2 convert target/thumbv8m.main-none-eabihf/release/pico-bridge \
    -t elf "$TMP/pico-good.uf2"
shasum -a 256 "$TMP/pico-good.uf2"   # must equal 659d9155…
```

Flash with the board in BOOTSEL:

```sh
picotool load -u -v -x "$TMP/pico-good.uf2"
```

Keep the USB cable connected after the reboot; the CDC port appears within a
couple of seconds.

## Temporary bench directory

`bench-temp/` is a visible, disposable directory inside the project for
one-off build and rollback artifacts. No temporary files are kept outside the
project or in hidden directories.

- `pico-precision-build-8f95b46/`: linked worktree at `8f95b46` used to build
  and boot-test the precision patch set; `backup-main-src/` holds the previous
  main-tree sources.
- `pico-known-good-8f95b46/`: linked worktree holding the verified plain
  known-good UF2 for the spare Pico.
- `main-switch-backup-2026-11-25/`: file copies and a patch made before moving
  the main tree back to `main`. All were restored successfully.

## Working firmware inventory

All known-good images are present inside the project. The binaries are ignored
by `*.uf2` / `*.bin` and are **not committed**; the hashes below are the record.

| Target | Artifact | SHA-256 | Status |
|---|---|---|---|
| Pico, active | `firmware/pico2w/fw/pico2w-dbg-fix.uf2` | `4c3263de16f80cdd66becb0b7d8a78be9cbc38fb448994742d4cbf51e50f6235` | Built from `c297027`, flashed and verified on 2026-11-25 |
| Pico, previous good | `firmware/pico2w/fw/pico2w-working.uf2` | `985f6e0a35edb43e753552ad035a49e25ba1f0be4db24493931219fc3883522b` | Verified rollback image |
| Pico, precision alt | `bench-temp/pico-precision-build-8f95b46/firmware/pico2w/precision-on-8f95b46.uf2` | `a851ad25439911f4be3a6dba20f641ab73773ecc38627efdfdcf99d5a8651516` | Same source, different build path; booted and verified before the main-tree build |
| Pico, plain known-good | `bench-temp/pico-known-good-8f95b46/firmware/pico2w/known-good.uf2` | `659d91554b3e1aa8b8177f568721bd809bd611502f000951bdbb90479fdefcb6` | On the spare Pico; verified rollback image |
| F103, active | `firmware/stm32f103/eferu-foc/eferu/build/precision-proto/hover.bin` | `267322b536509ecd6fc34e4d21baa1056f86196a6a23e0ea75e0178ca8b00dbe` | 40408 bytes, `DEBUG_SERIAL_PROTOCOL` enabled, flashed and verified with OpenOCD |
| F103, previous good | `firmware/stm32f103/eferu-foc/eferu/build/precision/hover.bin` | `fcf938732606e888a8742db24d98a9392c62a301d53a43ea5f41da381fe04883` | 34272 bytes, output-only debug console |
| F103, pinned rollback | `firmware/stm32f103/eferu-foc/eferu/build/hover.bin` | `056ec69151f9dbc4edd092eb08dabe2a2a7afa9b0eeddf9d10c59cb490a047df` | Pinned C build, not overwritten by the precision build |

## Current bench state

| Board | Chip ID | Image | Evidence |
|---|---|---|---|
| Main Pico | `0x4cc40d29dc728687` | `4c3263de…` (built from `c297027`) | CDC after ~1.2 s, `ok ping`, `stat` with `tmax=5.0 al=0 ar=0`, `ok dbg tx 13` confirms the full-line debug forwarding |
| Spare Pico | `0x7cb6b2c5535e6820` | `659d9155…` | CDC after ~1.2 s, `ok ping`, full `stat` |

The mainboard is powered on: `mb=ok`, `bat=36.37` V, `fb_ok=1`, `fb_drop=0`,
`rx` increasing. The IMU is live (`imu=ok`). The drive stays safe with
`estop=1`, `armed=0`, `L=0 R=0`, `al=0 ar=0`. The ST-Link is connected but idle.
No balance trial was run. The F103 debug protocol is live: `dbg $GET ANG_MEAS`
returns `ANG_MEAS=0` and `dbg $GET FILT` returns `FILT=19661`.

After a USB re-enumeration the applied-command feedback was checked with
`test 10 10` and `test 0 0`: `al`/`ar` followed both requests, with 114 ms
rise and 57 ms fall in the command-reporting path. A subsequent dead-zone probe
up to 60 units gave non-repeatable, asymmetric wheel motion, so no `db` value
was changed. A one-wheel spin-up identification gave 210–241 units/Nm in the
usable 60/80-unit steps, close to the compiled 200 units/Nm; `tmax` was left at
5.0 Nm. The bench was returned to `estop=1`, `armed=0`, `L=R=0`, `al=ar=0`.
See `docs/precision-control-2026-11-25.md`.

With the pinned C firmware the mainboard goes to sleep when the motors have
not been driven for a while. While asleep it sends no feedback frames and no
USART3 debug output: Pico `stat` shows `mb=none`, `bat=0.00`, `fb_ok=0`, a
stopped `rx` counter and `age=4294967295`. A brief wheel command after `clear`
wakes it; this is not a Pico link failure. The state was observed after the
2026-11-25 Pico reflash while the wheels stayed idle.

## Source-level analysis of the regression

The commit range `8f95b46..221dc37` changes only three Pico firmware files
(`console.rs`, `main.rs`, `state.rs`, 78 insertions): it adds the runtime
`protocol` switch and the hardened frame encoder/decoder. The core crate is
byte-identical across that range; `Cargo.lock`, `build.rs` and `memory.x` are
also identical. The current working tree has already removed the hardened
protocol code from `main.rs`, so its control path is essentially the known-good
one plus the four precision-patch lines (`tmax`, applied-command reporting).

Rebuilding `8f95b46` today produces a different ELF from the 09:06 known-good
ELF, but the difference is a crate-disambiguator hash (`CsgqFfHmL57dF` vs
`Cs4hVq5V32uCR`) and the resulting code layout only. `.rodata`, `.data`, the
vector table and the RP2350 boot2 block are identical, `rustc` fingerprints
match, and both images declare `image type: ARM Secure`. So the build
environment is not an obvious culprit and the small source delta remains the
leading hypothesis.

The practical way forward is to port the precision patches onto the proven
`8f95b46` tree instead of debugging the broken `221dc37` lineage. That keeps a
known-good boot path and avoids repeating the failed flashes.

## Precision patches on the known-good base (built, not yet flashed)

A new worktree `bench-temp/pico-precision-build-8f95b46` is checked out at `8f95b46` and
carries only the precision patches, ported to the flat source layout:

- `pico_core/src/eferu.rs`: feedback words 1/2 become `applied_left_units` /
  `applied_right_units`, with checksum, codec and tests updated.
- `pico_core/src/control.rs`: test for the inverse `units_per_nm` relationship.
- `pico_core/src/predictor.rs`: removes a pre-existing `unused_mut` that made
  clippy fail on the old commit; no behaviour change.
- `pico2w/src/state.rs`, `main.rs`, `console.rs`: runtime `tmax`, applied-command
  telemetry (`al`/`ar`) and the F103-side `ANG_MEAS`/`FILT` knobs remain on the
  mainboard side.

Artifact: `bench-temp/pico-precision-build-8f95b46/firmware/pico2w/precision-on-8f95b46.uf2`,
SHA-256 `a851ad25439911f4be3a6dba20f641ab73773ecc38627efdfdcf99d5a8651516`.
Cross-check, clippy, 137 core tests and the release build pass.

This image was flashed to the main Pico on 2026-11-25 and **does boot**: CDC
`/dev/cu.usbmodemJBOT_PICO_011` appeared about 1.2 s after reboot, `ping`
returned `ok ping`, `tmax` returned `ok tmax 5.0`, and `stat` ended with
`tmax=5.0 al=0 ar=0`. This proves the USB regression belongs to the `221dc37`
lineage, not to the precision patches, the cable, or the board. The mainboard
was powered off during this check, so `fb_ok=0` is expected.

## Main tree restored to the working source

`firmware/pico2w` now carries the same source as `bench-temp/pico-precision-build-8f95b46`:
the `protocol`/`hardened` code was removed from `fw/src/console.rs`,
`fw/src/state.rs` and `core/src/lib.rs`, and every `.rs` file is byte-identical
to the proven worktree. The previous main-tree sources were backed up to
`bench-temp/pico-precision-build-8f95b46/backup-main-src/`.

Build artifact: `firmware/pico2w/fw/pico2w-working.uf2`, SHA-256
`985f6e0a35edb43e753552ad035a49e25ba1f0be4db24493931219fc3883522b`. Its hash
differs from the worktree artifact only because the crate disambiguator depends
on the source path; the source is identical. Cross-check, clippy, 137 core tests
and the release build pass.

This main-tree artifact was flashed to the main Pico on 2026-11-25 and
**verified**: CDC appeared about 1.2 s after reboot, `ping` → `ok ping`, `tmax`
→ `ok tmax 5.0`, `help` lists `tmax`, and `stat` ends with
`tmax=5.0 al=0 ar=0`. The mainboard was powered off, so `fb_ok=0` is expected.

## Do not

- Do not reflash the Pico with any of the known-bad images "to try again".
- Do not assume the precision patches are lost: they are still in the working
  tree. They must be ported onto the `8f95b46` base or the boot regression must
  be fixed first.
