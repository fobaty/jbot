# MOE gate — HIL checklist and sign-off

MOE is the only path that energizes the motor bridge. In this workspace it is
compiled behind the `moe` feature of `f103_fw`; without that feature the reset
state (MOE low) is never changed. Enabling it requires all of the following.

## Build commands

```sh
cd firmware/stm32f103/rust/tools/calibration
./flash.sh --flash              # MOE-off image (default)
./flash.sh --flash --moe        # MOE-enabled image — ONLY after sign-off below
```

Both images call `hal::set_output_enable()` only from one place in the FOC ISR,
and with the `moe` feature it still requires: `ARMED == true`, `CAL_READY ==
true`, and both motor diagnostics clear. Any fault clears MOE on the next tick.

## Preconditions

- [ ] Wheels lifted; nothing can touch the floor.
- [ ] Current-limited bench supply or a known-good pack; sideboards disconnected.
- [ ] ST-Link on SWDIO/PA13, SWCLK/PA14, GND; button held during the write.
- [ ] USB-UART on PB10/PB11 at 3.3 V for the console (not PA2/PA3).
- [ ] Scope or logic analyzer on TIM8/TIM1 gate pins and PA4 buzzer.
- [ ] Rollback available: the C firmware is in git at upstream `4f141cb` plus
      `firmware/stm32f103/eferu-jbot.patch`.

## MOE-off checks (run on the default image)

| # | Check | Method | Pass condition |
|---|---|---|---|
| 1 | Probe | `st-info --probe` | `chipid != 0`, voltage is a number |
| 2 | Boot | `$HELP`, `$GET` | ticks rise, `cal=1`, `proto=LEGACY8`, `mode=FREEWHEEL`, `cyc/max/miss` within 4000-cycle budget |
| 3 | ADC | `$ADC`, `$DUMP` | packed field order; six currents near learned offset; pack/temp plausible |
| 4 | Halls | Spin wheels by hand, `$TRACE 160` CSV | hall code cycles 1..6; `speed_rpm` sign matches direction; `diag=0` |
| 5 | Link | Pico at 100 Hz legacy feedback | `rej` stable; no link-timeout fault while the Pico is sending |
| 6 | Timeout | Disconnect USART2 | after 800 ms `fault=LINK_TIMEOUT`, torque demand goes to zero |
| 7 | Recovery | Reconnect + fresh command + `$ARM` | fault clears, controller leaves OPEN mode |
| 8 | Tuning | `$GETP`, `$SET IQKP 1500`, `$SAVE`, power cycle | value visible in `$TRACE`, `nvm=FLASH`, `$SAVE` refused while armed |
| 9 | Buttons/LED/buzzer | Power button, boot, short press | LED on at boot; startup sweep; short press powers off |
| 10 | Trace load | `$TRACE 160` under all of the above | `miss` and `TRACE_DROPPED` remain zero |

## MOE-on checks (only after the table above passes)

- [ ] Flash the `moe` image with the button held.
- [ ] With the wheels still lifted and the controller **disarmed**, verify with
      the scope that no gate switches (MOE low).
- [ ] `$ARM`, then verify phase order and dead time against the reference:
      16 kHz centre-aligned, ARR 2000, dead time 48, U/V/W mapping TIM1/TIM8.
- [ ] Verify current sign: a positive `iq` demand must produce the expected
      phase-current polarity on the shunts.
- [ ] Verify duty scaling and both current limits via `$GETP`/`$SET`.
- [ ] Step torque demand in small increments and confirm `iq` follows in
      `$TRACE` without oscillation.
- [ ] Confirm all three safe modes and the watchdog reset path.
- [ ] Record DWT timing (`cyc/max/miss`) under load.

## Sign-off

MOE may be enabled only when every box above is checked and the following is
filled in. The operator signing here acts as Safety-Reviewer for this bench
session.

- HIL date: ____________
- Operator / Safety-Reviewer: ____________
- Scope: bench, wheels lifted  /  loaded robot (circle one)
- Result of each MOE-on check: ____________
- GPL-3.0 derivative-work decision: ____________

Until this is signed, the shipped image is the MOE-off build and the robot
cannot produce torque.
