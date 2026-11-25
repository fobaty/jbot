# JBOT wiring — consolidated connections

This is the single cross-board wiring table. Board-level pinouts are in
`../firmware/stm32f103/mainboard_pinout.png` and
`../firmware/pico2w/pico2w-pinout.svg`; hardware rationale is in
[`hardware.md`](hardware.md) and the link contract in
[`pico-bridge.md`](pico-bridge.md).

## 1. Overview

```
Raspberry Pi 5 ──(GP6/GP7, 460800)──┐
XR1 receiver ───(GP4/GP5, CRSF)─────┤
BNO055 ────────(GP2/GP3, I2C1)──────┼── Pico 2 W ──(GP16/GP17, 115200)── STM32F103 ── motors, halls, battery
USB console ─────────────────────────┤        │                         │
Logic analyzer ───(GP10 marker)──────┘        │                         └── ST-Link (SWD, programming)
                                              └──(GP12/GP13, 115200)── STM32 USART3 debug (right cable)
```

The two hoverboard cables carry GND, 12/15 V and two signal wires. The left
cable is the Pico torque link; the right cable is the STM32 debug bridge. The
sideboards are disconnected.

## 2. All external links

| # | Link | From | To | Interface / rate | Where | State |
|---|---|---|---|---|---|---|
| 1 | Torque command | Pico **GP16** (UART0 TX) | STM32 **PA3** (USART2 RX) | 115200 8N1, 8-byte frame | left cable | Active |
| 2 | Mainboard feedback | STM32 **PA2** (USART2 TX) | Pico **GP17** (UART0 RX) | 115200 8N1, 18-byte frame | left cable | Active |
| 3 | Debug bridge RX | STM32 **PB10** (USART3 TX) | Pico **GP12** (PIO UART RX) | 115200 8N1 | right cable | Active when `dbg on` |
| 4 | Debug bridge TX | Pico **GP13** (PIO UART TX) | STM32 **PB11** (USART3 RX) | 115200 8N1 | right cable | Active when `dbg on` |
| 5 | IMU | Pico **GP2** SDA / **GP3** SCL | BNO055 SDA / SCL | I2C1, 400 kHz, module pull-ups | on-robot | Active |
| 6 | Radio | XR1 TX / RX | Pico **GP5** RX / **GP4** TX | UART1, 420 kbaud CRSF | on-robot | Planned, not implemented |
| 7 | Mission computer | Pico **GP6** TX / **GP7** RX | RPi **GPIO15** RX / **GPIO14** TX | PIO UART, 460800 8N1 | on-robot | Planned, not implemented |
| 8 | Balance timing marker | Pico **GP10** | logic analyzer | GPIO toggle per 200 Hz step | bench | Active |
| 9 | Charge detect (candidate) | STM32 **PA12** | Pico **GP11** | GPIO / event | on-robot | Open item |
| 10 | Programming | ST-Link SWDIO / SWCLK / GND | STM32 **PA13** / **PA14** / GND | SWD, no NRST wire | bench | Active |
| 11 | Pico console | host USB | Pico USB CDC-ACM | 115200 | bench | Active |
| 12 | Sideboards | left/right cable signal wires | sideboard MCUs | proprietary UART | — | **Disconnected** |

The torque link originally used GP0/GP1. It was reworked to **GP16/GP17** in
November 2026; GP0/GP1 are spare. Do not use the old mapping.

## 3. STM32F103 pin quick reference

| Pin(s) | Function | Use / note |
|---|---|---|
| PA2 / PA3 | USART2 TX / RX | Left cable: feedback out / command in. Also ADC2 inputs in dual mode. Not 5 V tolerant |
| PB10 / PB11 | USART3 TX / RX (or I2C2) | Right cable: debug bridge to Pico GP12/GP13; stage 2 hip link. 5 V tolerant |
| PA13 / PA14 | SWDIO / SWCLK | ST-Link. No NRST wire: hold the power button during a write |
| PA1 | Power button | Active high, soft input |
| PA4 | Buzzer | Active-high gate |
| PB2 | Status LED | Lit after the startup melody |
| PA5 | Power latch | Held high by firmware; release cuts board power |
| PA12 | Charge detect | Candidate line to Pico GP11 |
| PB5 / PB6 / PB7 | Left hall sensors U/V/W | Active low |
| PC10 / PC11 / PC12 | Right hall sensors U/V/W | Active low |
| PC6, PC7, PC8 + PA7, PB0, PB1 | Left TIM8 PWM U/V/W + complements | Gate drive |
| PA8, PA9, PA10 + PB13, PB14, PB15 | Right TIM1 PWM U/V/W + complements | Gate drive |
| PC2 | Battery voltage ADC | Divider input |
| PC0, PC1, PC3, PC4, PC5, PA0 | Phase/DC current ADC inputs | Dual ADC1/ADC2 sampling |

## 4. Pico 2 W pin quick reference

| GPIO | Function | Peer | Note |
|---|---|---|---|
| GP16 / GP17 | UART0 TX / RX | STM32 PA3 / PA2 | Torque link, active |
| GP2 / GP3 | I2C1 SDA / SCL | BNO055 | 400 kHz |
| GP4 / GP5 | UART1 TX / RX | XR1 receiver | CRSF, planned |
| GP6 / GP7 | PIO UART TX / RX | RPi GPIO15 / GPIO14 | Mission link, planned |
| GP10 | GPIO out | logic analyzer | Balance-step marker |
| GP11 | GPIO in | STM32 PA12 | Charge detect candidate |
| GP12 / GP13 | PIO0 UART RX / TX | STM32 PB10 / PB11 | Debug bridge, `dbg on` |
| GP0 / GP1 | spare | — | Former torque link, unused |
| USB | CDC-ACM | host | Console and flashing |

## 5. Safety rules

- All Pico GPIO are **3.3 V and not 5 V tolerant**. The left cable (PA2/PA3) is
  also 3.3 V only; the right cable is 5 V tolerant but is wired at 3.3 V here.
- The 12/15 V wire in each cable stays live while the board is on. Identify it
  with a multimeter and insulate it; it must never touch logic.
- Both sideboards must stay disconnected: they transmit on the same two signal
  wires that now carry the Pico links.
- The power button is only a firmware input. The hardware kill switch in the
  battery line is the real E-stop.
- For SWD use SWDIO, SWCLK and GND only. There is no NRST wire, so hold the
  mainboard power button during the whole write.
