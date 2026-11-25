# EFeru FOC reference firmware (JBOT snapshot)

This directory groups the pinned upstream C firmware with the JBOT patches. It
is the behavioural reference and rollback artifact for the Rust torque node in
`../rust`.

| Path | Content |
|---|---|
| `eferu/` | Upstream checkout at commit `4f141cbc97b297e194fd58e56444bd0322f902ac`, with `eferu-config.patch` already applied to `Inc/config.h`. It keeps its own `.git` pointing at the upstream repository; **do not push there**. |
| `eferu-config.patch` | Current `Inc/config.h` delta against the pinned commit. |
| `eferu-jbot-config.h` | Full snapshot of the flashed `Inc/config.h`, byte-identical to the patched file. |
| `eferu-jbot.patch` | Earlier revision of the configuration delta, kept for history. Do not apply it on top of the current snapshot. |
| `eferu-bench.patch` | Bench-only Makefile `unlock` target fix for ST-Link clones; not applied to the flashed source. |
| `eferu-balance.patch` | Planned production torque-node delta; not created yet. |

Upstream is GPL-3.0; `eferu/LICENSE` is included. Build and flash instructions
are in `../README.md`. The Makefile flow used for the flashed image is:

```
cd eferu
make -e VARIANT=VARIANT_USART
```
