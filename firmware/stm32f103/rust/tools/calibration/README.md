# Host calibration tools

These scripts drive the STM32F103 Rust torque node over the USART3 debug
console (115200 8N1). They are dependency-free: the serial port is configured
with Python's standard-library `termios`, so no `pip install` is needed on
macOS or Linux.

## Files

- `f103_trace.py` - host mirror of the hardened COBS/CRC16 frame codec and the
  88-byte FOC trace payload. Run `python3 f103_trace.py --selftest` before
  connecting hardware; it checks the CRC check value, both protocol golden
  vectors, a full trace round trip and a chunked-frame demultiplexer.
- `f103_bridge.py` - serial CLI for `$HELP`, `$GET`, `$GETP`, `$SET`, `$SAVE`,
  `$ARM`, `$DISARM`, `$ADC`, `$DUMP`, `$INIT`, `$SET PROTOCOL` and `$TRACE`.
- `flash.sh` - build, `objcopy`, size check and `st-flash`. Defaults to a dry
  run; pass `--flash` to actually write the image.

## Build and flash

```sh
./flash.sh                 # build + verify, no write
./flash.sh --flash         # build + verify + st-flash --reset write 0x8000000
```

The script refuses an image larger than the 252 KiB application slot so the
two top flash pages holding the NVM record cannot be overwritten. If the chip
is readout protected, unlock first:

```sh
openocd -f interface/stlink-v2.cfg -f target/stm32f1x.cfg \
  -c init -c "reset halt" -c "stm32f1x unlock 0"
```

MOE is never enabled by this firmware, so flashing and all checks below are
torque-free. Enabling gate drive requires HIL validation and an explicit
Safety-Reviewer sign-off.

## Console workflow

```sh
python3 f103_bridge.py --port /dev/cu.usbserial-XXXX help
python3 f103_bridge.py --port /dev/cu.usbserial-XXXX get
python3 f103_bridge.py --port /dev/cu.usbserial-XXXX getp
python3 f103_bridge.py --port /dev/cu.usbserial-XXXX set IQKP 1500
python3 f103_bridge.py --port /dev/cu.usbserial-XXXX save
python3 f103_bridge.py --port /dev/cu.usbserial-XXXX trace --decim 160 --seconds 10 --out run1.csv
python3 f103_bridge.py --port /dev/cu.usbserial-XXXX arm      # bench only
python3 f103_bridge.py --port /dev/cu.usbserial-XXXX disarm
```

`$TRACE` clamps any non-zero decimation below 160 to 160, giving at most 100 Hz
at 115200 8N1. The CSV has one row per frame with the tick, sequence, timestamp,
flags, DC currents, six duty values and both motors' internal state (mode,
scheduler, error code, angle, speed, target, iq/id, vq/vd, phase amplitudes and
the full 32-bit PI integrators).

## MOE-off HIL checklist

1. `$HELP`, `$GET`: ticks increment, `cal=1`, `proto=LEGACY8`, `nvm=DEFAULT`,
   and `cyc`/`max`/`miss` stay inside the 4000-cycle budget.
2. `$ADC`, `$DUMP`: packed field order, currents around the learned offset,
   battery and temperature plausible.
3. Spin the wheels by hand: hall code, `speed_rpm` and direction sign in
   `$TRACE`; `diag=0`.
4. Pico link: legacy feedback at 100 Hz with `rej` stable; then switch both
   sides to hardened with `$SET PROTOCOL HARDENED`.
5. `$GETP` -> `$SET IQKP 1500` -> `$TRACE 160`: confirm the new gain is visible
   in the trace; `$SAVE`, power cycle, `$GET nvm=FLASH`. `$SAVE` must refuse
   while armed.
6. `$ARM` (bench): the generated torque loop runs and `iq`/integrators/duty
   respond to a target, while MOE stays low - verify no gate switching on the
   scope and no wheel movement. `$DISARM` returns to OPEN mode.
7. Disconnect USART2: after 100 ms the safe state ramps torque to zero and a
   link-timeout fault is latched; a fresh valid command plus `$ARM` recovers.
8. `$TRACE 160` under load: `miss` and `TRACE_DROPPED` remain zero.

Only after these pass, and after Safety-Reviewer sign-off and a decision on the
GPL-3.0 derivative obligation, may a separate, reviewed change enable MOE.
