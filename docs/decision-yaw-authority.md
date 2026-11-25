# Yaw-authority decision (WP2, 2026-11-22)

## Context

The flashed mainboard runs the upstream EFeru `VARIANT_USART` build. In that
variant `STEER_COEFFICIENT` has no override, so it falls back to
`DEFAULT_STEER_COEFFICIENT = 8192`, i.e. 0.5
(`firmware/stm32f103/eferu-foc/eferu-jbot-config.h:202,688-690`; `platformio.ini:48-66`).
`SPEED_COEFFICIENT` stays 1.0. The 16384/16384 (both 1.0) override belongs to
`VARIANT_PWM`, not to the flashed image.

The Pico inverse `Command::from_wheel_torque`
(`firmware/pico_core/src/eferu.rs:74`) computes `speed = (L+R)/2` and
`steer = (L-R)/2`, assuming a unit steer coefficient. The mainboard mixer
(`Src/util.c:1706-1722`, called from `Src/main.c:361`) applies
`cmdL = speed + 0.5*steer` and `cmdR = speed - 0.5*steer`.

## Effect

A requested wheel pair `(L, R)` therefore becomes
`cmdL = (L+R)/2 + (L-R)/4`, `cmdR = (L+R)/2 - (L-R)/4`. For `L=100, R=-100`
the mainboard applies `+50/-50` instead of `+100/-100`: yaw differential is
half of the request. Common-mode balance torque is unaffected because it
travels on `speed`, whose coefficient is 1.0.

## Options

1. **Change the Pico inverse to `steer = L - R`.** With coefficient 0.5 this
   reproduces the requested wheel pair exactly while `|L-R| <= 1000`. Beyond
   that the wire `steer` field saturates at +/-1000, so maximum spin-in-place
   torque is +/-500 per wheel instead of +/-1000. No mainboard reflash; a
   later Pico reflash is required before bench trials.
2. **Set the mainboard `STEER_COEFFICIENT` to 16384 (1.0) and reflash with the
   power button held.** Keeps the Pico code unchanged and restores full yaw
   authority, but is the more invasive reflash and the mainboard link has
   already shown power/enumeration problems.

## Recommendation

Option 1. It makes yaw proportional and correct for the balance and spin
commands actually used, and avoids a mainboard reflash. The +/-500 per-wheel
spin ceiling is acceptable for a controlled low-torque pivot; option 2 can be
done later as a separate approved reflash if full-authority spin is needed.

## Status

Option 1 approved and implemented (2026-11-22). `Command::from_wheel_torque`
now sends the full wheel-torque difference; the mixer round-trip unit test
covers both the corrected mapping and the +/-500 per-wheel saturation ceiling.
The code was flashed to the Pico on 2026-11-22 (`picotool load -u -v -x -t elf`,
verified OK). The mainboard config is unchanged. The yaw response itself is
still unmeasured on the bench because the mainboard was not woken during the
reflash session.
