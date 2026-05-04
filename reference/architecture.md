# AGV System Architecture

## Overview

The `AGV/` repo contains the software and firmware of a custom AGV (Autonomous Guided Vehicle).

**Traction:** Differential drive via two identical 12V DC motors.

**Processing units:**
| Unit | Role |
|---|---|
| STM32F0Discovery | Main MCU — real-time control, sensor I/O, motor driving |
| ESP32-C3 SuperMini | Wireless bridge — hosts a Wi-Fi AP, relays commands/telemetry between workstation and STM32 over UART |
| NVIDIA Jetson Orin Nano | High-level compute — computer vision + LiDAR (future scope, not active) |

---

## Power Supply

Two separate LiPo batteries power the system. All grounds are shared.

| Rail | Source | Consumers |
|---|---|---|
| 12V | 3S LiPo → step-down to 12V | DC motors via Pololu G2 |
| 5V | 3S LiPo → step-down to 5V | STM32, ESP32-C3, sensors |
| 3.3V | 3S LiPo → step-down to 3.3V | STM32 I/O logic, peripherals |
| 19V | 6S LiPo → step-down to 19V | Jetson Orin Nano only |

---

## Operating Modes

The AGV is always in exactly one **mode**. The mode determines whether a heartbeat/regular command stream from the workstation is expected.

| Mode | Description |
|---|---|
| `SUPERVISED` | A workstation connection is active and a periodic heartbeat is expected. |
| `UNSUPERVISED` | AGV operates autonomously. No heartbeat expected. Some functions unavailable. |

### Heartbeat timeout (SUPERVISED → UNSUPERVISED)
Loss of heartbeat triggers a **two-stage graceful degradation**:

1. **Immediately on timeout:** transition to `UNSUPERVISED`. Any `SUPERVISED`-only function (e.g. `REMOTE_CONTROL`) is replaced by `STANDBY`. The caution modifier baseline for `UNSUPERVISED` mode applies.
2. **3 seconds later (if connection not restored):** virtual E-STOP is triggered. Requires explicit workstation clear to recover.

The heartbeat timeout duration is configurable in `config.h`.

---

## Functions

Functions define what the AGV is actively doing. Most are mutually exclusive in the **Navigation** group; background functions run continuously regardless.

### Navigation functions (mutually exclusive — pick one)

| Function | Supervised | Unsupervised | Description |
|---|---|---|---|
| `STANDBY` | Yes | Yes | Motors idle, system active, waiting for a command. Default state. |
| `REMOTE_CONTROL` | Yes | No | Workstation sends velocity/direction commands directly. |
| `LINE_FOLLOW` | Yes | Yes | Follows a line on the floor using the QTR-8A sensor array. |
| `TRAJECTORY_FOLLOW` | Yes | Yes | Follows a pre-loaded sequence of waypoints using odometry. |

### Background functions (always active, run in parallel with any navigation function)

| Function | Description |
|---|---|
| `OBSTACLE_DETECTION` | Proximity sensors monitored via EXTI. Triggers E-STOP on detection; auto-clears when obstacle is gone. |
| `CARGO_MONITORING` | Continuous independent load cell reads. Triggers caution modifier or E-STOP on imbalance/overload. |
| `ODOMETRY` | Encoder counts integrated continuously for position/velocity estimation. |
| `CURRENT_MONITORING` | ADC reads of motor current sense pins for overcurrent fault detection. |
| `HEARTBEAT_WATCH` | In `SUPERVISED` mode: monitors for workstation heartbeat timeout. |

---

## E-STOP System

### Physical E-STOP
Physical E-STOP buttons cut power directly to the Pololu G2 motor driver (e.g. via a relay or contactor on the 12V motor rail). The MCU remains powered and can detect the loss of driver power, log the event, and manage recovery — but it cannot override a physical E-STOP in software. Physical E-STOP is the ultimate hardware safety guarantee.

### Virtual E-STOP
A software flag that commands the MCU to assert the `SLEEP` pins on both motor channels of the Pololu G2, disabling the driver outputs without cutting power to it. Slower to respond than a physical cut but fully controllable and recoverable by firmware or workstation command. Can be triggered by:

| Source | Auto-clears? | Workstation override? |
|---|---|---|
| Proximity sensor (obstacle detected) | Yes — clears when all proximity sensors deassert | Yes — workstation can force-clear early |
| Cargo overload or severe imbalance (load cells) | Yes — clears when weight/imbalance returns within limits | Yes |
| Heartbeat timeout (3 s grace expired) | No — requires reconnection + explicit clear | Yes |
| Workstation command | No — requires explicit clear from workstation | Yes |
| Firmware fault (overcurrent, watchdog, etc.) | No — requires explicit reset | Yes |

### E-STOP Priority
Physical E-STOP (power cut to driver) always takes precedence and **cannot be overridden by software or the workstation** — it is a physical safety guarantee. Virtual E-STOP (SLEEP pin assertion) is a software layer that operates independently; both can be active simultaneously. Recovery from a non-auto-clearing virtual E-STOP requires all active non-auto-clearing sources to be resolved (or workstation-overridden) and a reset to be issued.

---

## Control Architecture

The state machine decides *which* navigation function is active. The actual driving — translating intent into motor commands — is handled by a **cascade controller**, with a per-function **control law** (navigator) producing the chassis-level setpoint.

```
state machine ──► navigator (control law) ──► cascade PID ──► caution gate ──► motors
```

### Navigators (control laws, not state machines)

Each navigation function has a navigator module that runs every control tick while its function is active. The navigator's only output is a chassis-level setpoint `(v_target, ω_target)`. Navigators are pure control laws — they do not transition between functions; the state machine handles that on receipt of a command.

| Function | Navigator output |
|---|---|
| `STANDBY` | `(0, 0)` |
| `REMOTE_CONTROL` | latest workstation `(v, ω)` (passthrough) |
| `LINE_FOLLOW` | `v_target` = constant cruise speed; `ω_target` = `PID(line_error)` from QTR-8A weighted centroid |
| `TRAJECTORY_FOLLOW` | `(v_target, ω_target)` = pure-pursuit law on next waypoint, advancing when within `WAYPOINT_REACH_RADIUS_M` |

Navigators may have their own internal PIDs (e.g. line-error → ω in `LINE_FOLLOW`); these are **separate** from the cascade controller and tuned per-navigator.

### Cascade controller

Two stacked control loops. Both run at `CONTROL_LOOP_HZ` (200 Hz default). Inner loop has independent gains per wheel to compensate for mechanical asymmetry.

```
OUTER LOOP (chassis)
   PI on linear:  v_target  − v_measured  (from odometry)  →  v_corrected
   PI on angular: ω_target − ω_measured (from odometry)  →  ω_corrected
   ↓
   kinematic split (differential drive):
     v_left_target  = v_corrected − (wheel_base/2) × ω_corrected
     v_right_target = v_corrected + (wheel_base/2) × ω_corrected
   ↓
INNER LOOP (per wheel, independent gains)
   PID on left  wheel: v_left_target  − v_encoder_left   →  pwm_left
   PID on right wheel: v_right_target − v_encoder_right  →  pwm_right
   ↓
   caution modifier × pwm  (single multiplication point — cannot be bypassed)
   ↓
   motors
```

The outer loop can be bypassed (`USE_OUTER_CASCADE 0` in `config.h`) for bring-up — the navigator's `(v, ω)` then feeds the kinematic split directly. This is useful for tuning inner-loop gains in isolation before layering the outer correction on top.

### Module-disable flags for debugging

Each subsystem's main-loop integration is wrapped in `#if !DISABLE_<MODULE>` and gated by a flag in `config.h` (default 0). To isolate a fault — say, suspecting the IMU is causing problems — flip `DISABLE_IMU 1`, rebuild, flash, and the main loop runs without ever reading the IMU. The rest of the system is unaffected.

This is preferred over runtime-toggleable flags: zero runtime cost, no risk of accidentally re-enabling a known-bad module, and grep-friendly (`grep DISABLE_ inc/config.h`) for verifying current build configuration.

---

## Caution Modifier

The **caution modifier** is a real-time scalar `[0.0, 1.0]` applied to the maximum allowed speed and acceleration. It is designed as a graceful degradation mechanism — a middle ground between full operation and E-STOP.

```
effective_max_speed  = configured_max_speed  × caution_modifier
effective_max_accel  = configured_max_accel  × caution_modifier
```

### Suggested levels

| Level | `caution_modifier` | Trigger examples |
|---|---|---|
| `NORMAL` | 1.0 | All clear |
| `CAUTION` | 0.5 | Mild load imbalance, unsupervised navigation uncertainty |
| `CRITICAL` | 0.2 | Severe imbalance, repeated proximity near-misses |
| `ESTOP` | 0.0 | E-STOP active (handled separately, modifier is informational here) |

### Sources and arbitration
Multiple sources can independently set a caution level. The system always uses the **most conservative (lowest) value** active at any time. Each source clears its own level when its condition resolves.

| Source | Condition | Suggested level |
|---|---|---|
| Load cells | Weight exceeds threshold but not E-STOP limit | `CAUTION` |
| Load cells | Severe imbalance (CoM too far from center) | `CRITICAL` |
| Mode | `UNSUPERVISED` + navigation function active | `CAUTION` (baseline) |
| Proximity | Sensor triggered but not yet E-STOP (future: range-based) | `CAUTION` |

### REMOTE_CONTROL behavior
The caution modifier is **active during `REMOTE_CONTROL`** — operator commands are still capped by `effective_max_speed`. The workstation can explicitly override it (set `caution_modifier = 1.0` from a super-user command), taking full responsibility. The override clears automatically when the triggering condition resolves.

### Implementation notes
- Store the modifier as a single `float` computed each control cycle by iterating over all source states and taking the minimum.
- All speed/PWM setpoint calculations read through this modifier before writing to hardware.
- The modifier value and active source bitmask should be included in telemetry sent to the workstation.
- Thresholds (imbalance limits, weight limits, etc.) belong in `config.h`.

---

## Workstation Authority

The workstation is a **super-user**. It can:
- Override any virtual E-STOP source (including proximity sensor auto-cleared ones, cargo faults, etc.)
- Override the caution modifier (force full speed temporarily)
- Force any mode or function transition, including ones the firmware would not perform autonomously
- Request an on-demand read of any sensor at any time, regardless of current function or polling schedule
- Update any runtime parameter live (see `PARAM_UPDATE` packet)
- Trigger a firmware soft-reset

The only thing the workstation **cannot** override is a physical E-STOP (button cutting power to the motor driver), since that is a physical circuit break.

---

## Sensor Polling Rates

Not all sensors need to be read continuously. Polling rates adapt to the current navigation function to avoid wasting CPU time and bus bandwidth.

| Sensor | STANDBY / Weight Setting | Moving (LINE_FOLLOW, TRAJECTORY_FOLLOW) | REMOTE_CONTROL | On-demand (workstation request) |
|---|---|---|---|---|
| Load cells (HX711) | 30 Hz | 2 Hz | 2 Hz | Always available |
| QTR-8A line sensors | Off | `LINE_FOLLOW` only: ~500 Hz; others: Off | Off | Yes — single burst read |
| Proximity sensors | Always (interrupt-driven EXTI, no polling) | Always | Always | N/A — interrupt-driven |
| Encoders | Always (hardware timer, no CPU cost) | Always | Always | N/A — hardware timer |
| Motor current sense | Off / periodic low-rate check | Active (~100 Hz) | Active | Always available |

> **"Weight setting period"** is a dedicated sub-state of `STANDBY` where the AGV is stationary and load cells are being actively read and averaged to establish a tare or reference weight distribution. It uses the same 30 Hz rate.

Polling rates are implemented via timer-gated flags in the main control loop, not blocking delays. All rates are configurable in `config.h`.

---

## STM32–ESP32 Communication Protocol

### Physical layer
- **Interface:** USART1, full-duplex
- **Baud rate:** 921600 bps (stable at STM32F0's 48 MHz system clock; balances throughput and error rate)
- **Direction:** Bidirectional. Both sides can initiate packets at any time.

### Packet frame

```
┌──────────┬──────────┬────────┬──────────┬─────────────────────┬──────────┐
│ MAGIC    │ SEQ      │ TYPE   │ LEN      │ PAYLOAD             │ CRC16    │
│ 2 bytes  │ 1 byte   │ 1 byte │ 1 byte   │ 0–255 bytes         │ 2 bytes  │
│ 0xAG 0xV │ 0x00–FF  │        │          │                     │ CCITT    │
└──────────┴──────────┴────────┴──────────┴─────────────────────┴──────────┘
  Total overhead: 7 bytes. Max frame: 262 bytes.
```

- **MAGIC:** `0xAA 0x56` — receiver scans for this to re-sync after framing errors.
- **SEQ:** Wrapping sequence number (0–255). Receiver detects dropped packets by gaps; sender detects lost ACKs by timeout.
- **TYPE:** Packet type byte (see below).
- **LEN:** Payload length in bytes (0 = no payload).
- **CRC16:** CRC16-CCITT computed over bytes `[SEQ, TYPE, LEN, PAYLOAD...]`. Receiver discards frames with CRC mismatch.

### Packet types (suggested)

| Type byte | Name | Direction | Description |
|---|---|---|---|
| `0x01` | `CMD` | WS→ESP→STM32 | Command: set function, set mode, E-STOP, clear |
| `0x02` | `PARAM_UPDATE` | WS→ESP→STM32 | Update one or more runtime parameters. Payload: one or more `[uint8_t param_id, float value]` (5 bytes each). |
| `0x03` | `TELEMETRY` | STM32→ESP→WS | Periodic state snapshot (speed, position, load cells, caution level, etc.) |
| `0x04` | `HEARTBEAT` | WS→ESP→STM32 | Workstation alive signal (empty payload) |
| `0x05` | `ACK` | Both | Acknowledge a received packet (payload = echoed SEQ + status byte) |
| `0x06` | `NACK` | Both | Reject a received packet (payload = echoed SEQ + error code) |
| `0x07` | `FRAG` | Both | Fragment of a larger logical message (payload = frag index + frag count + data) |
| `0xFF` | `RESET` | WS→STM32 | Request firmware soft-reset or E-STOP clear |

### Reliability
- **ACK/NACK:** `CMD` and `PARAM_UPDATE` packets require an `ACK` within a timeout (suggested: 50 ms). Sender retries up to N times (suggested: 3) before flagging a comms fault.
- **Telemetry and heartbeat** are fire-and-forget (no ACK required).
- **Fragmentation:** If a logical message exceeds 255 bytes, use `FRAG` packets. Each fragment carries its own CRC. The receiver reassembles in order; missing fragments are NACKed by index.

### ESP32 role
The ESP32 is a transparent relay with minimal processing:
- Receives Wi-Fi UDP/TCP packets from the workstation, wraps them in the above frame format, and forwards over UART to the STM32.
- Receives UART frames from the STM32 and forwards them to the workstation over Wi-Fi.
- Handles the CRC check on UART frames (discards corrupt frames, sends NACK).
- Manages the Wi-Fi AP and client connection state.

---

## Hardware

All peripherals are connected to the STM32 and operate on **3.3V logic**.

| # | Component | Qty | Notes |
|---|---|---|---|
| a | Pololu G2 24V14 | 1 | Dual DC motor driver |
| b | CZL601 load cell + HX711 amplifier | 4 each | 100 kg capacity per cell; one per corner of cargo platform |
| c | Pololu QTR-8A | 1 | 8-channel analog reflectance / line sensor |
| d | OMRON E6B2 incremental encoder | 2 | Channels A and B only (no index). Default: 600 PPR — see `config.h` |
| e | E18-D80NK proximity sensor | 4 | NPN digital output; one per side (front, rear, left, right) |
| f | BNO055 IMU | 1 | 9-DOF IMU (accelerometer, gyroscope, magnetometer + onboard sensor fusion). Read via I2C1. 3.3V. |

---

## STM32 Pin Mapping

### 1. UART — ESP32-C3 Communication
| Pin | Function |
|---|---|
| PB6 | USART1_TX |
| PB7 | USART1_RX |

### 2. Quadrature Encoders (hardware timer mode)
| Pin | Function |
|---|---|
| PA0 | Encoder 1 Phase A (TIM2_CH1) |
| PA1 | Encoder 1 Phase B (TIM2_CH2) |
| PA6 | Encoder 2 Phase A (TIM3_CH1) |
| PA7 | Encoder 2 Phase B (TIM3_CH2) |

### 3. Pololu G2 Dual Motor Driver
| Pin | Function |
|---|---|
| PA8 | Motor 1 PWM (TIM1_CH1) |
| PA2 | Motor 1 Current Sense (ADC_IN2) |
| PB0 | Motor 1 DIR |
| PB1 | Motor 1 SLEEP |
| PA11 | Motor 2 PWM (TIM1_CH4) |
| PA3 | Motor 2 Current Sense (ADC_IN3) |
| PB2 | Motor 2 DIR |
| PB3 | Motor 2 SLEEP |

### 4. QTR-8A Reflectance Sensor Array (analog)
| Pin | Function |
|---|---|
| PA4 | QTR Sensor 1 (ADC_IN4) |
| PA5 | QTR Sensor 2 (ADC_IN5) |
| PC0 | QTR Sensor 3 (ADC_IN10) |
| PC1 | QTR Sensor 4 (ADC_IN11) |
| PC2 | QTR Sensor 5 (ADC_IN12) |
| PC3 | QTR Sensor 6 (ADC_IN13) |
| PC4 | QTR Sensor 7 (ADC_IN14) |
| PC5 | QTR Sensor 8 (ADC_IN15) |

### 5. E18-D80NK Proximity Sensors (EXTI interrupts)
| Pin | Function | Facing |
|---|---|---|
| PC6 | Proximity Sensor 1 (EXTI6) | Front |
| PC7 | Proximity Sensor 2 (EXTI7) | Rear |
| PC8 | Proximity Sensor 3 (EXTI8) | Left |
| PC9 | Proximity Sensor 4 (EXTI9) | Right |

> Facing assignments are logical labels defined in `config.h` and may change to match physical mounting.

### 6. BNO055 IMU (I2C1)
| Pin | Function |
|---|---|
| PB8 | I2C1_SCL (AF1) |
| PB9 | I2C1_SDA (AF1) |

> PB6/PB7 are the primary I2C1 pins but are occupied by USART1. PB8/PB9 are the valid alternate mapping (AF1) and are free. External pull-up resistors required (4.7 kΩ to 3.3V). BNO055 I2C address: `0x28` (ADR pin low, default) or `0x29` (ADR pin high).

### 7. HX711 Load Cell Amplifiers (bit-banged)
| Pin | Function |
|---|---|
| PB10 | Shared SCK / Clock (output) |
| PB12 | HX711 Data 1 — Front-left corner (input) |
| PB13 | HX711 Data 2 — Front-right corner (input) |
| PB14 | HX711 Data 3 — Rear-left corner (input) |
| PB15 | HX711 Data 4 — Rear-right corner (input) |

> All four HX711s share the SCK line and are read simultaneously (all DOUT lines sampled in parallel per clock pulse), which is valid per the HX711 datasheet and keeps read time deterministic. Corner assignments are logical labels defined in `config.h` and may change to match physical wiring.

---

## Repository Layout

```
AGV/
├── firmware/
│   ├── stm32/       # STM32CubeIDE project (bare metal C)
│   └── esp32/       # PlatformIO project (Arduino framework)
├── software/        # Code for the Jetson or the remote workstation
└── reference/       # Style guides, conventions, implementation notes
```

---

## Coding Guidelines

**Priority order:** Reliability → Performance → Everything else.

- Avoid race conditions, undefined behavior, and edge cases by design — do not paper over them.
- **STM32:** Bare metal only. Use STM32 CMSIS macros (no magic bit shifts or raw addresses). Prefer hardware modules (DMA, hardware encoder timers, ADC scan mode, etc.) over software polling to minimize CPU load. Hardware-configurable parameters (PPR, weight thresholds, speed limits, PID gains, etc.) belong in a top-level `config.h`.
- **ESP32:** PlatformIO + Arduino framework preferred.
- **Jetson / workstation:** No specific constraints defined yet.
