# STM32F103 mainboard firmware (JBOT bench profile)

This directory records the configuration that is actually flashed on the
STM32F103RCT6 mainboard. The upstream EFeru sources are kept in
`eferu-foc/eferu/` at commit `4f141cb`. That nested repository has `origin` set
to `https://github.com/EFeru/hoverboard-firmware-hack-FOC.git`; **do not push
there**. The JBOT patches live next to it in `eferu-foc/`:

- `eferu-foc/eferu-config.patch` — current diff of `Inc/config.h` against the
  pinned commit.
- `eferu-foc/eferu-jbot-config.h` — full snapshot of the flashed `Inc/config.h`;
  byte-identical to the patched file in the working tree.
- `eferu-foc/eferu-jbot.patch` — earlier revision of the config change, kept for
  history. Do not apply it on top of the current snapshot.
- `eferu-foc/eferu-bench.patch` — bench-only Makefile `unlock` target fix; it is
  not applied to the flashed source.

When the mainboard is reflashed, update the config snapshot and patch in the
same commit.

## Flashed configuration (reflashed 2026-11-24)

| Parameter | Value | Note |
|---|---|---|
| Build variant | `VARIANT_USART` | Passed on the command line with `make -e VARIANT=VARIANT_USART` |
| Control type | `FOC_CTRL` | Upstream default |
| Control mode | `TRQ_MODE` | Torque commands from the Pico bridge |
| `I_MOT_MAX` | `10` A | Conservative limit for the original 10s2p 4.4 Ah pack |
| `I_DC_MAX` | `12` A | Kept at `I_MOT_MAX + 2 A` |
| `N_MOT_MAX` | `400` rpm | Bench speed ceiling |
| `SERIAL_TIMEOUT` | `160` ticks (about 0.8 s) | `160 x 5 ms`; about 2 s to a full stop including wheel coast; the 100 ms target is not met |
| USART2 | `FEEDBACK_SERIAL_USART2` | Left cable, PA2/PA3, 115200 8N1, 8-byte command / 18-byte feedback frames |
| Debug serial | `DEBUG_SERIAL_USART3` | Right cable, PB10/PB11, 115200 8N1; prints the boot configuration on startup. The Pico PIO bridge on GP12/GP13 forwards it to the USB console (`dbg on`). The `DEBUG_SERIAL_USART2` line added by the patch is inside `VARIANT_PWM` and does not apply to `VARIANT_USART` |
| Inputs | `PRI_INPUT1/2 = 3, -1000, 0, 1000, 0` | Serial input, no deadband |
| `FLASH_WRITE_KEY` | `0x1002` | `VARIANT_USART`, no dual inputs |
| `FILTER` | `19661` | 0.3 (α), tau ~14 ms; reflashed 2026-11-23, previously 6553 / 0.1 |
| `SPEED_COEFFICIENT` | `16384` | 1.0 |
| `STEER_COEFFICIENT` | `8192` | 0.5 from `DEFAULT_STEER_COEFFICIENT`; `VARIANT_USART` has no override. The Pico mixer inverse now compensates for 0.5 (WP2, not yet flashed); differential saturates at +/-1000 units. Do not change this without revisiting the Pico side. |
| `TANK_STEERING`, `INVERT_L/R_DIRECTION` | Not defined | Stock mixer and motor directions |

Do not raise `I_MOT_MAX` towards 15 A without measured MOSFET and pack
temperatures. The bench profile is deliberately below the upstream defaults.

## Restoring the configuration

From the repository root:

```
cp firmware/stm32f103/eferu-foc/eferu-jbot-config.h firmware/stm32f103/eferu-foc/eferu/Inc/config.h
```

Alternatively, start from the pinned `Inc/config.h` and apply
`firmware/stm32f103/eferu-foc/eferu-jbot.patch`. Do not apply the patch on top of the
full snapshot.

## Building

```
cd firmware/stm32f103/eferu-foc/eferu
make -e VARIANT=VARIANT_USART
```

The build produces `build/hover.bin`. A PlatformIO environment
`VARIANT_USART` also exists but the Makefile flow is the one used for the
flashed image.

## Flashing (wiki Method 4, macOS CLI)

Hardware: ST-Link V2 (verified with firmware V2J37S7), connected with
**SWDIO, SWCLK and GND only**. There is no NRST wire.

1. Power the mainboard and **hold its power button during the whole write**.
   The STM32 releases the power latch on reset; without the button held,
   `st-flash` reports `NRST is not connected`, `Soft reset failed: error
   write to AIRCR` and `Failed to connect to target`.
2. Run:

```
cd firmware/stm32f103/eferu-foc/eferu
make flash
```

   This executes `st-flash --reset write build/hover.bin 0x8000000`.
3. Success looks like `Flash written and verified!`; release the button.
4. Confirm the Pico bridge reports `fb_ok=1` after the board boots.

Do not use `st-flash --connect-under-reset` with this wiring: NRST is absent,
so that path fails. Reference:
https://github.com/EFeru/hoverboard-firmware-hack-FOC/wiki/Compiling-and-flashing-the-firmware
(Method 4: MacOS CLI).

## Safety notes

- The mainboard enters sleep after about 8 minutes idle when the command
  magnitude stays at or below 50 units. Hold the power button for 2-3 s to
  wake it.
- Loss of the serial link zeroes the command after roughly 0.8 s plus wheel
  coast-down. Keep the rope and hands on the body during every powered test.
- The flashed image is torque mode. A wrong sign or an unlocked current limit
  can make the wheels accelerate without warning.
