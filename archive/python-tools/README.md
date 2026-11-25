# Archived Python tooling

Everything in this directory is **legacy reference material**. The project no
longer builds, tests or uses Python tools. The active stack is:

* Pico 2 W firmware: `firmware/pico2w/fw` (Rust, its own Cargo workspace);
* bench console: `crates/jbot_console` (Rust);
* offline analysis: `crates/jbot_analysis` (Rust).

The files are kept only so an old procedure can be reconstructed if needed.
They are not covered by the Rust quality gates, and their paths in the docs
were updated when they were moved here.

## Contents

| Path | Runs on | Original role |
|---|---|---|
| `bench_paths.py` | host | Resolves bare filenames into `logs/` and `csv/`. Its repository-root lookup was updated for the `archive/python-tools` depth. |
| `pico/jbot_link.py` | host | USB CDC protocol layer, telemetry/stat parsing, event log and CSV writer. |
| `pico/jbot_console.py` | host | curses console with a `--no-ui` mode; superseded by `crates/jbot_console`. |
| `pico/jbot_drive.py` | host | curses drive console; superseded by the Drive panel of the Rust console. |
| `pico/jbot_menu.py` | host | Plain-text drive menu; superseded by the Rust console. |
| `pico/jbot_torque_id.py` | host | Spin-up torque-scale identification via `M = I*alpha`. Prints to stdout; it does **not** write a CSV. |
| `pico/test_jbot_link.py` | host | Unit tests for the old protocol layer. |
| `hover/hover_bridge.py` | Pico 2 W | MicroPython bridge owning the 20 Hz motor frame stream; alternative to the Rust firmware. |
| `hover/hover_drive.py` | host | Console for `hover_bridge.py`. |
| `hover/hover_menu.py` | Pico 2 W | Older self-contained MicroPython menu; writes its log on the Pico filesystem. |

## Using an archived tool

Do not use these tools while the Rust firmware is running: both stacks want to
own the same USB serial port. To run one, flash the corresponding MicroPython
image first and adapt the example commands in `pico/README.md` or
`hover/README.md` to the paths above. Results written to `logs/` and `csv/`
still land in the real repository root because `bench_paths.py` resolves them
relative to the root, not to this archive.
