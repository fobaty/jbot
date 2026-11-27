# Automatic identification and parameter recommendation

## Purpose

`identify_all` is a host-side driver in `crates/jbot_console`. It measures the
drive and balance plant through the existing Pico USB console and prints a
concrete set of runtime commands for the compensation knobs that already exist
in the firmware but are off at boot (`cal`, `db`, `dither`, `ff`, `pscale`,
`dscale`, `slew`).

It does not flash firmware, change a safety contract, or arm the robot. Every
phase ends with `home 0` and `estop`.

## Phases

| Phase | What it measures | Gate |
|---|---|---|
| `baseline` | Link, IMU freshness, feedback age, upright start | Refuses an externally supported or moving start |
| `spin` | Units/Nm from free-wheel acceleration `M = I * alpha`; per-wheel breakaway | Rejects a plateau with no usable acceleration |
| `hold` | Settled common-mode holding torque around upright (feedforward curve) | Guards on estop, fall latch, 250 dps, 0.4 m |
| `search` (opt-in) | Bounded `pscale`/`dscale`/`slew` ladder | Rejects a profile that does not settle, stops at first guard |

Phenomena that need external instruments are reported as manual, not guessed:
phase current waveform and dead time (oscilloscope), continuous winding torque
(thermocouple), low-speed velocity quality (encoders), and repeatable release
(fixture).

## Usage

From the workspace root, with the robot secured in the test zone:

```sh
cargo run -p jbot_console --bin identify_all -- --run --secured
```

Useful options: `--port PATH`, `--out-dir csv`, `--levels 39,48,60,75,99,120,150`,
`--angles -4,-2,0,2,4`, `--hold-ms 600`, `--search`, `--trial-ms 4000`.

For a robot fixed on a stand with both wheels free in the air, add `--spin-only`.
It runs the spin phase and skips the balance phases, which require floor
contact. Do not combine it with `--search`.

Run `--help` for the full list. `--secured` is mandatory; the driver refuses
to energise without it.

## Outputs

- `csv/jbot-identify-<epoch>.csv` - all telemetry with a phase label.
- `csv/jbot-identify-<epoch>.json` - fitted values, confidence and commands.

Confidence is `measured` only when spin, hold and a profile search all
succeeded. A missing phase degrades confidence to `partial` or `unverified`;
the driver leaves the corresponding compensation off rather than inventing a
value.

## Safety boundary

This tool only sends existing console commands and reads telemetry. It never
writes flash, never changes watchdogs, limits or protocol fields, and never
treats a guarded run as a successful measurement. A physical raise trial or a
change to the flashed defaults still requires separate approval.
