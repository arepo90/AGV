# AGV System Architecture

## Overview

The `AGV/` repo contains the firmware and software of a custom AGV (Autonomous
Guided Vehicle): a differential-drive robot with two 12 V DC motors.

**Processing units:**

| Unit | Role |
|---|---|
| STM32F051 (F0-Discovery class) | Main MCU — real-time control, sensor I/O, motor driving. Bare-metal C at 48 MHz. |
| ESP32-C3 SuperMini | Transparent USB-CDC ↔ UART byte pump to the Jetson; drives the WS2812B indicator rings + onboard LED from a telemetry tap. |
| NVIDIA Jetson Orin Nano | High-level compute — ROS 2 bridge (UART ↔ topics) + Wi-Fi bridge to the workstation; future LiDAR consumer. |

The workstation laptop connects to a Wi-Fi AP hosted by the Jetson (or a shared
router). The browser GUI is served by `python3 -m http.server 8080` from the
workstation and reaches the firmware over a WebSocket to the Jetson.

---

## Repository Layout

```
AGV/
├── firmware/
│   ├── STM32/         # STM32 firmware — bare-metal Makefile + CMSIS (GCC, J-Link)
│   │   └── src/
│   │       ├── config.h types.h main.c
│   │       ├── hal/<module>/<module>.{c,h}   # pseudo-HAL: talks to silicon only
│   │       └── app/<module>/<module>.{c,h}   # libraries built on the HAL
│   └── ESP32/             # PlatformIO project (Arduino framework)
├── software/
│   ├── GUI/               # Browser GUI, served by `python3 -m http.server 8080`
│   └── bridge/            # colcon workspace (ROS 2 Humble)
│       └── src/
│           ├── agv_msgs/    # custom msgs + srvs (ament_cmake)
│           └── agv_bridge/  # uart_bridge_node + ws_bridge_node (rclpy)
└── reference/             # this document + conventions
```

The STM32 firmware builds with `cd firmware/STM32 && make` (artifacts in
`build/`, flash with `make flash` via J-Link). The bridge builds with
`colcon build` in `software/bridge/`.

---

## Power Supply

Two separate LiPo batteries power the system; all grounds are shared.

| Rail | Source | Consumers |
|---|---|---|
| 12V | 3S LiPo | DC motors via Pololu G2 |
| 5V | 3S LiPo → step-down | STM32, ESP32-C3, some sensors |
| 3.3V | 5V → step-down | Some sensors |
| 19V | 6S LiPo → step-down | Jetson Orin Nano |

Pack voltages are monitored by two INA219 (bus-voltage sense only; the shunt is
unused): 0x40 on the 3S rail, 0x41 on the 6S rail. The 3S reading feeds the
battery low-voltage monitor; the 6S is display-only (the STM32 cannot protect the
Jetson rail). Battery percentage is derived downstream (GUI + Jetson panel node)
from the raw millivolts via a per-cell LiPo curve.

---

## Operating Modes

The AGV is always in exactly one **mode**, which determines whether a heartbeat
from the workstation is expected.

| Mode | Description |
|---|---|
| `SUPERVISED` | A workstation connection is active; a periodic heartbeat is expected. |
| `UNSUPERVISED` | The AGV operates autonomously. No heartbeat expected; some functions unavailable. |

### Heartbeat timeout (two-stage graceful degradation)

1. **On timeout** (`HEARTBEAT_TIMEOUT_MS`, default 1000 ms of silence): transition
   to `UNSUPERVISED`. Any `SUPERVISED`-only function (`REMOTE_CONTROL`) drops to
   `STANDBY`. The `UNSUPERVISED` caution baseline applies.
2. **Grace expiry** (`HEARTBEAT_GRACE_MS`, +3000 ms): virtual E-STOP fires. Recovery
   requires reconnection **and** an explicit workstation clear.

Any inbound packet — not just `HEARTBEAT` — counts as proof of life and refreshes
the watchdog.

---

## Functions

Functions define what the AGV is actively doing. Navigation functions are mutually
exclusive; background monitors always run.

### Navigation functions (pick one)

| Function | Supervised | Unsupervised | Description |
|---|---|---|---|
| `STANDBY` | Yes | Yes | Motors idle (SLEEP asserted), system active. Default. |
| `REMOTE_CONTROL` | Yes | No | Workstation sends `(v, ω)` velocity commands directly. |
| `LINE_FOLLOW` | Yes | Yes | Follows a floor line via the QTR-8A array. |
| `TRAJECTORY_FOLLOW` | Yes | Yes | Pure-pursuit over a pre-loaded waypoint polyline using odometry. |

### Background monitors (always active)

| Monitor | Description |
|---|---|
| `OBSTACLE_DETECTION` | IR proximity sensors via EXTI. Triggers E-STOP on detection; auto-clears when clear. |
| `TOF_RANGING` | 4 VL53L0X distance sensors via I2C mux. Distance bands → caution level, then auto-clearing E-STOP. |
| `CARGO_MONITORING` | Load-cell reads → caution or E-STOP on overload / imbalance. |
| `ODOMETRY` | Encoder + IMU fusion for pose/velocity (see below). |
| `CURRENT_MONITORING` | Motor current-sense ADC → overcurrent E-STOP. |
| `BATTERY_MONITORING` | 2 INA219 read 3S/6S bus voltage. 3S undervoltage → caution → auto-clearing E-STOP (hysteresis); 6S display-only. |
| `HEARTBEAT_WATCH` | In `SUPERVISED` mode: monitors for heartbeat timeout. |

---

## E-STOP System

A software flag asserts the `SLEEP` pins on both Pololu G2 channels, putting the
driver outputs in Hi-Z without cutting its power. `main.c` drives `SLEEP` LOW
(motors de-energised) whenever `estop_active()` is true **or** the function is
`STANDBY`. During active navigation without an E-STOP, `SLEEP` is HIGH and braking
is handled by PWM.

| Source | Auto-clears? | Workstation override? |
|---|---|---|
| Proximity (IR obstacle) | Yes — when all sensors deassert | Yes — can force-clear early |
| TOF (close range) | Yes — when nearest distance leaves the E-STOP band | Yes |
| Cargo overload / imbalance | Yes — when within limits | Yes |
| Battery low (3S) | Yes — when voltage recovers above trip + hysteresis | Yes |
| Heartbeat grace expired | No — needs reconnect + explicit clear | Yes |
| Workstation command | No — needs explicit clear | Yes |
| Motor overcurrent | No — needs explicit clear | Yes |
| Firmware fault | No — needs explicit clear | Yes |

The source bitmask is 9 bits wide (was 7); `estop_sources`/`caution_sources` are
carried as 16-bit fields in `TLM_CORE`. E-STOP source assert/clear is
interrupt-safe (the proximity ISR calls into it).

---

## Caution Modifier

A real-time scalar in `[0.0, 1.0]` that throttles the AGV as a middle ground
between full operation and E-STOP:

```
effective_max_speed = configured_max_speed × caution_modifier   (chassis clamp)
effective_max_accel = configured_max_accel × caution_modifier   (inside the ramp)
```

| Level | Modifier | Example trigger |
|---|---|---|
| `NORMAL` | 1.0 | All clear |
| `CAUTION` | 0.5 | Mild imbalance; TOF mid-range; 3S low; UNSUPERVISED navigation baseline |
| `CRITICAL` | 0.2 | Severe imbalance / overload; TOF close range |
| `ESTOP` | 0.0 | E-STOP active (modifier is informational here) |

Multiple sources can each set a level; the firmware uses the **minimum** (most
conservative). When the workstation sends an explicit override (`PKT_CMD 0x06`),
that value is used directly with full authority — it can be more permissive or
more restrictive than the firmware minimum.

---

## Control Architecture

The state machine selects the active navigation function. Each control tick
(`CONTROL_LOOP_HZ`, default **100 Hz**) runs this chain in `app/control`:

```
navigator → ramp (slew/shape) → caution clamp → kinematic split → per-wheel PI+FF → motors
```

1. **Navigator** (`app/nav`) produces a chassis setpoint `(v_target, ω_target)`.
2. **Ramp** (`app/ramp`) slews/shapes the setpoint so the wheels never see a step;
   its `max_accel`/`max_jerk` are scaled by the caution modifier.
3. **Caution clamp** bounds `v`/`ω` to `MAX_*_SPEED × caution_modifier`.
4. **Kinematic split** (algebraic differential drive — robot geometry assumed
   correct, no closed-loop chassis correction):
   ```
   v_left  = v − (WHEEL_BASE/2)·ω
   v_right = v + (WHEEL_BASE/2)·ω
   ```
5. **Per-wheel velocity PI + feedforward** (`app/pid`, independent gains per wheel):
   ```
   duty = clamp( Kff·v_target + Kp·e + Ki·∫e dt , [-1, +1] )
   ```
   No derivative term (encoder velocity is too quantised to differentiate). The
   feedforward carries the steady-state duty; the integrator only trims. Anti-windup
   is conditional integration — the integrator holds when the output is saturated
   unless the error is driving it back in range. On E-STOP the controllers and ramp
   reset so motion resumes without windup.

### Navigators

| Function | Output |
|---|---|
| `STANDBY` | `(0, 0)` |
| `REMOTE_CONTROL` | latest workstation `(v, ω)` (passthrough) |
| `LINE_FOLLOW` | `v` = cruise (halved on sharp corrections); `ω` = `PID(line_error)` from the QTR weighted centroid. This is the only PID in the firmware. |
| `TRAJECTORY_FOLLOW` | pure pursuit over the waypoint polyline (below) |

#### Pure-pursuit navigator (`TRAJECTORY_FOLLOW`)

Path = straight segments between waypoints, plus an implicit "waypoint 0" captured
from the pose at start so cross-track error is defined from tick 0. Per tick:
project the pose onto the active segment, walk `PURE_PURSUIT_LOOKAHEAD_M` forward
along the path to a lookahead point `L`, then
`α = wrap_pi(atan2(L−pose) − θ)`, `κ = 2·sin(α)/‖pose−L‖`,
`v = cruise / (1 + TRAJECTORY_CURV_SLOWDOWN·|κ|)`, `ω = clamp(v·κ, ±ω_max)`.
If `L` is behind the robot (`|α| > π/2`) it rotates toward `L` at half ω_max rather
than reversing. Completes within `WAYPOINT_REACH_RADIUS_M` of the final waypoint.

### Motion profile (ramp)

A chassis-level slew limiter between the navigator and the kinematic split, applied
uniformly to all navigators. Four runtime-selectable shapes:

| Shape | Parameters | Behaviour |
|---|---|---|
| `LINEAR` | `max_accel` | Constant max acceleration (trapezoidal `v` on a step). |
| `SCURVE` | `max_accel`, `max_jerk` | Jerk-limited; acceleration itself ramps. |
| `EXPONENTIAL` | `τ` | 1st-order filter `v += (1−e^{−dt/τ})·(v_cmd−v)`. |
| `CUSTOM` | operator curve `f(s):[0,1]→[0,1]` | Piecewise-linear (≤8 pts), uploaded at runtime; segment duration scales so the steepest part tops out at `max_accel`. |

Linear and angular axes have independent magnitudes; the shape is shared. The
caution modifier scales `max_accel`/`max_jerk` at step time. Reset on E-STOP.

---

## Odometry — heading fusion (complementary filter)

`app/odometry` owns pose. Heading θ fuses the IMU and encoders with one tunable
and no covariance matrices:

```
ω_rate = gyro_z                if IMU present + gyro calibrated   (slip-immune)
       = (v_r − v_l)/WHEEL_BASE otherwise                          (encoder fallback)
θ      = θ + ω_rate·dt
θ      = wrap(θ + (1−HEADING_COMP_ALPHA)·wrap(yaw_BNO055 − θ))     if yaw calibrated
v      = ½(v_l + v_r);   x += v·cos θ·dt;   y += v·sin θ·dt
```

Rotation comes from the BNO055 gyro (immune to wheel slip); a slow complementary
pull toward the BNO055 absolute yaw bounds long-term drift. With no IMU it degrades
to pure encoder odometry. Position is dead-reckoned (no sensor observes it directly
yet; when LiDAR/SLAM arrives it can be promoted to a filtered state). Consumed by
telemetry and the pure-pursuit navigator.

`IMU_HEADING_SIGN` reconciles the BNO055 yaw convention (CW-positive) with the
encoder math convention (CCW-positive), applied inside `app/imu`.

---

## Module-Disable Flags

Each subsystem's main-loop integration and init is wrapped in `#if !DISABLE_<MODULE>`
gated by a flag in `config.h` (default 0 = present). Flip one to 1 to bring sensors
up one at a time on the bench without editing `main.c`. Zero runtime cost,
grep-friendly (`grep DISABLE_ config.h`).

---

## STM32 ↔ ESP32 ↔ Jetson Communication Protocol

The same binary frame runs end-to-end. The ESP32 forwards bytes without inspecting
them; the Jetson's `uart_bridge_node` is the framing authority on the host side.

### Physical layer
- **STM32 ↔ ESP32:** USART1, full-duplex DMA, **921600 bps**.
- **ESP32 ↔ Jetson:** native USB-CDC on the C3 (`/dev/agv-esp32` via udev rule).
- **Jetson ↔ workstation:** Wi-Fi → WebSocket binary frames on `ws://<jetson>:8765/ws`.
- Bidirectional; either endpoint may initiate.

### Frame format

```
┌────────┬─────┬─────┬──────┬─────┬──────────────┬────────┐
│ MAGIC  │ VER │ SEQ │ TYPE │ LEN │ PAYLOAD      │ CRC16  │
│ AA 56  │ 02  │ ..  │ ..   │ ..  │ 0–255 bytes  │ CCITT  │
└────────┴─────┴─────┴──────┴─────┴──────────────┴────────┘
  8-byte overhead. CRC-16/CCITT over [VER, SEQ, TYPE, LEN, PAYLOAD...].
```

- **VER** is `0x02`. Receivers reject mismatched versions and re-sync on `MAGIC`.
- **SEQ** wraps 0–255 for gap detection.
- Frames with a CRC mismatch are discarded (and NACKed by the firmware).

### Packet types

| Byte | Name | Direction | Description |
|---|---|---|---|
| `0x01` | `CMD` | WS→STM32 | Command (sub-typed; see below) |
| `0x02` | `PARAM_UPDATE` | WS→STM32 | One or more `[u8 id][f32 value]` tuples |
| `0x03` | `TLM_CORE` | STM32→WS | Operational state + pose (fast) |
| `0x04` | `HEARTBEAT` | WS→STM32 | Workstation alive signal (empty) |
| `0x05` | `ACK` | both | `[echoed_seq, status]` |
| `0x06` | `NACK` | both | `[echoed_seq, error_code]` |
| `0x08` | `LOG` | STM32→WS | One fault-log entry |
| `0x09` | `TLM_DRIVE` | STM32→WS | Per-wheel control internals |
| `0x0A` | `TLM_SENSORS` | STM32→WS | Load cells + IMU orientation |
| `0x0B` | `TLM_QTR` | STM32→WS | QTR raw + line position |
| `0xFF` | `RESET` | WS→STM32 | `[]`/`0x00` = clear E-STOP, `0x01` = soft reset |

### Reliability
- `CMD` and `PARAM_UPDATE` are ACK/NACKed; the workstation retries on timeout.
- Telemetry, log, and heartbeat are fire-and-forget.

### Telemetry streams

Telemetry is split into four rate-grouped streams instead of one large frame, so
each consumer reads only what it needs and slow data isn't re-packed fast. Each
stream is its own packet type (the ESP32 tap filters on `TLM_CORE` cheaply). All
fields little-endian; layouts are fixed by `app/telemetry`.

**`TLM_CORE`** — 42 bytes:

| Off | Field | Type |
|---|---|---|
| 0 | `timestamp_ms` | u32 |
| 4 | `mode` | u8 |
| 5 | `function` | u8 |
| 6 | `estop_sources` | u16 (bitmask; bit7 TOF, bit8 battery) |
| 8 | `caution_sources` | u16 (bitmask; bit5 TOF, bit6 battery) |
| 10 | `caution_modifier` | f32 |
| 14 | `flags` | u8 (bit0 adc, bit1 hx711, bit2 imu, bit3 tare) |
| 15 | `velocity_linear` | f32 |
| 19 | `velocity_angular` | f32 |
| 23–34 | `odom_x`, `odom_y`, `odom_theta` | 3×f32 |
| 35 | `current_left_ma` | u16 |
| 37 | `current_right_ma` | u16 |
| 39 | `proximity_obstructed` | u16 (bits 6–9 = PC6–PC9) |
| 41 | `led_mode` | u8 (indicator-ring animation: 0 pulse, 1 snake) |

**`TLM_DRIVE`** — 32 bytes: `v_left_target`, `v_right_target`, `velocity_left`,
`velocity_right`, `duty_left`, `duty_right` (6×f32), then `encoder_left_counts`,
`encoder_right_counts` (2×u32).

**`TLM_SENSORS`** — 41 bytes: `load_cells[4]` (FL, FR, RL, RR kg, 4×f32),
`imu_yaw_deg`, `imu_pitch_deg`, `imu_roll_deg` (3×f32), `imu_calib` (u8, packed
`sys<<6|gyro<<4|accel<<2|mag`), then `tof_mm[4]` (Front, Rear, Left, Right; 4×u16)
and `batt_3s_mv`, `batt_6s_mv` (2×u16, 0 if absent).

**`TLM_QTR`** — 20 bytes: `qtr_raw[8]` (8×u16) + `line_position` (f32, [-1,+1]).
Sent only during `LINE_FOLLOW` / QTR calibration.

**`LOG`** — 12 bytes: `timestamp_ms` (u32), `code` (u16), `severity` (u8),
`module` (u8), `data` (u32).

### Stream rates (config.h)

| Stream | Moving | Idle |
|---|---|---|
| `TLM_CORE` | 50 Hz | 10 Hz |
| `TLM_DRIVE` | 20 Hz | 20 Hz |
| `TLM_SENSORS` | 5 Hz | 5 Hz |
| `TLM_QTR` | control rate (LINE_FOLLOW / cal only) | off |

### ESP32 role
A transparent byte pump: USB-CDC `Serial` ↔ Jetson, `HardwareSerial(1)` ↔ STM32 at
921600. No protocol interpretation either way. A side-tap parses completed
`TLM_CORE` frames to drive the indicator LED rings (state colour + `led_mode`
animation) and the onboard status LED — its only duty besides forwarding.

### Jetson role
ROS 2 Humble workspace `software/bridge/` with two `agv_bridge` nodes:
- **`uart_bridge_node`** owns `/dev/agv-esp32`, parses the protocol, and is the
  single ROS-side authority. Publishes the four streams as `agv_msgs/Tlm{Core,Drive,Sensors,Qtr}`
  on `/agv/{core,drive,sensors,qtr}`, plus standard `nav_msgs/Odometry` on `/agv/odom`
  and `sensor_msgs/Imu` on `/agv/imu` (derived from CORE/SENSORS) for rviz/SLAM, and
  `/agv/log`. Inbound it accepts `/agv/cmd_vel`, `/agv/heartbeat`, `/agv/param_update`,
  and exposes one service per `CMD` subtype + `RESET` variant for ACK-confirmed commands.
- **`ws_bridge_node`** serves `ws://<jetson>:8765/ws`, forwarding each telemetry
  stream verbatim to the GUI and routing inbound GUI frames to the matching
  uart_bridge service/publisher. End-to-end heartbeat semantics are preserved.
- **`panel_node`** drives the Arduino UNO status display (below): it subscribes to
  `/agv/core` + `/agv/sensors`, derives battery percentage, and writes a compact,
  checksummed ASCII status line to `/dev/agv-uno` at a few Hz. One-way, display-only.

Custom messages/services live in the `agv_msgs` package.

---

## Indicator LED Rings (ESP32)

Four WS2812B rings driven from the ESP32 replace a traditional stack light. Each
ring is wired as a loop (first/last LED adjacent) on its own data GPIO (0–3); two
large rings (120 LEDs) and two small (30). All four show the same state in the
stack-light colours.

The state colour follows the system, derived from each `TLM_CORE` frame at the tap:

| Condition | Colour |
|---|---|
| `estop_sources != 0` | **red** (ESTOP) |
| `caution_sources != 0` | **yellow** (CAUTION) |
| both zero | **green** (NORMAL) |
| no telemetry since boot | **red**, slow (INIT) |
| telemetry stopped > 2 s | **red**, slow (DISCONNECT) |

The **animation style** is operator-selectable from the GUI and arrives in the
`led_mode` byte of `TLM_CORE` (`PARAM_LED_MODE`, 0 = pulse, 1 = snake):

- **Pulse** — the whole ring breathes (half-rectified cosine), faster for ESTOP.
- **Snake** — a comet circles the ring; a normalised phase keeps the 120- and
  30-LED rings visually synchronised.

Animations are computed from `millis()` so they stay phase-coherent across state
changes; rendering is rate-limited (the WS2812B `show()` briefly blocks).

## Status LED (onboard ESP32)

Encodes mode + function (non-ESTOP): a mode flash (100 ms SUPERVISED / 400 ms
UNSUPERVISED), pause, then N×100 ms flashes where N = function ID (STANDBY=0 …
TRAJECTORY_FOLLOW=3), on a ~1.2 s cycle. ESTOP: continuous 100 ms on/off.

## Status Panel (Arduino UNO + 3.5" TFT)

A display-only Arduino UNO with a parallel 3.5" TFT shield (no touch) hangs off the
Jetson over USB (`/dev/agv-uno`, `99-agv.rules`). The Jetson's `panel_node` pushes a
compact, newline-terminated ASCII status line at ~5 Hz:

```
AGV,<mode>,<func>,<estop>,<caution>,<cm>,<b3>,<p3>,<b6>,<p6>,<t0>,<t1>,<t2>,<t3>*<csum>
```

`cm` = caution modifier ×100; `bN` = pack mV; `pN` = charge % (−1 absent); `tN` = TOF
mm (Front, Rear, Left, Right); `csum` = 8-bit XOR of the body, two hex digits. The
sketch (`firmware/UNO/`, MCUFRIEND_kbv + Adafruit_GFX) validates the checksum, renders
mode / function / a NORMAL·CAUTION·E-STOP banner / 3S+6S battery bars / TOF footer, and
shows **NO LINK** if the feed stops for >2 s. The link is one-way; the UNO never talks
back. Battery percentage is computed on the Jetson from the raw millivolts (shared
LiPo curve with the GUI).

Everything operationally meaningful is reachable through the binary protocol — the
workstation is the single source of truth for everything the firmware does not own
(controller gains, load-cell scales, runtime params, trajectory waypoints). It is a
super-user: it can override any E-STOP source, set the caution modifier in either
direction, force any mode/function transition, update any runtime parameter, and
trigger a soft reset.

### Commands (`PKT_CMD` sub-types)

| Sub-type | Name | Payload | When allowed |
|---|---|---|---|
| `0x01` | `SET_FUNCTION` | `[u8 func]` | mode-legal only |
| `0x02` | `SET_MODE` | `[u8 mode]` | always |
| `0x03` | `VEL_CMD` | `[f32 v][f32 ω]` | acted on in REMOTE_CONTROL |
| `0x04` | `VIRTUAL_ESTOP` | — | always |
| `0x05` | `OVERRIDE_ESTOP_SOURCE` | `[u16 mask]` (LE; high byte optional) | always |
| `0x06` | `OVERRIDE_CAUTION` | `[f32 scalar]` | always |
| `0x08` | `LOAD_TRAJECTORY` | `[u8 op]` (0 clear / 1 append `[f32 x][f32 y]`) | not in TRAJECTORY_FOLLOW |
| `0x09` | `START_TARE` | — | STANDBY only |
| `0x0A` | `LOG_DUMP_REQUEST` | — | always |
| `0x0B` | `LOG_CLEAR` | — | always |
| `0x0C` | `QTR_CALIBRATE` | `[u8 op]` (0 begin / 1 save+persist / 2 cancel / 3 defaults) | STANDBY only |
| `0x0D` | `RESET_ODOMETRY` | — | STANDBY only |
| `0x0E` | `LOAD_RAMP_CURVE` | `[u8 op]` (0 begin / 1 add `[f32 s][f32 f]` / 2 commit / 3 cancel) | always |

### Runtime parameters (`PKT_PARAM_UPDATE` IDs)

| ID | Parameter |
|---|---|
| `0x03` / `0x04` | max linear / angular accel (ramp) |
| `0x10`–`0x12` | left wheel `Kp` / `Ki` / `Kff` |
| `0x13`–`0x15` | right wheel `Kp` / `Ki` / `Kff` |
| `0x20`–`0x22` | line-follow `Kp` / `Ki` / `Kd` |
| `0x23` | line cruise speed (m/s) |
| `0x24` / `0x25` / `0x27` | trajectory cruise / lookahead / curvature-slowdown |
| `0x26` | QTR line-lost threshold |
| `0x30`–`0x33` | weight caution / E-STOP (kg), imbalance caution / E-STOP (fraction) |
| `0x34`–`0x37` | HX711 per-corner offset (counts) |
| `0x38`–`0x3B` | HX711 per-corner scale (counts→kg) |
| `0x40` | ramp shape (0 LINEAR / 1 SCURVE / 2 EXPONENTIAL / 3 CUSTOM) |
| `0x41` / `0x42` | ramp jerk linear / angular (SCURVE) |
| `0x43` / `0x44` | ramp τ linear / angular (EXPONENTIAL) |
| `0x50` | indicator-ring animation (0 pulse, 1 snake) |
| `0x60`–`0x62` | TOF caution / critical / E-STOP distance (mm) |
| `0x63` / `0x64` | 3S battery caution / E-STOP voltage (mV) |

All param updates are applied immediately and stored in RAM only — the workstation
persists values and re-sends them on connect.

### Calibration

- **Tare:** STANDBY + empty platform → `START_TARE`; `flags` bit 3 clears when done.
- **Per-corner scale:** after tare, place a known weight, read `load_cells[i]`,
  compute `new_scale = W_ref / (raw − offset)`, send via `PARAM_UPDATE 0x38+i`.
- **QTR sweep:** STANDBY → `QTR_CALIBRATE 0`, sweep the array over the line ~3–5 s,
  then `1` to commit (per-sensor min/max written to the last flash page with magic +
  CRC, loaded on boot). `2` cancels, `3` reverts to defaults + erases. A sensor with
  range < 200 counts keeps its previous baseline.

### Requires a firmware rebuild
Pin remapping, adding a function/navigator, protocol/telemetry layout changes, and
anything in `config.h` without a `PARAM_*` ID (max speeds, telemetry rates, polarity
flags, `DISABLE_*` toggles, heading-filter `α`).

---

## Sensor Polling Rates

Rates adapt to the active function and are configured in `config.h`; all are
timer-gated flags in the main loop, never blocking delays.

| Sensor | STANDBY | Moving | REMOTE_CONTROL |
|---|---|---|---|
| Load cells (HX711) | 30 Hz | 2 Hz | 2 Hz |
| QTR-8A | off | LINE_FOLLOW: control rate; else off | off |
| Proximity (IR) | EXTI (interrupt) | EXTI | EXTI |
| TOF (VL53L0X) | 60 Hz aggregate (round-robin, ~15 Hz/sensor) | same | same |
| Battery (INA219) | 2 Hz | 2 Hz | 2 Hz |
| Encoders | hardware timer | hardware timer | hardware timer |
| Motor current (ADC) | 100 Hz | 100 Hz | 100 Hz |
| IMU (BNO055) | 100 Hz | 100 Hz | 100 Hz |

Control loop: **100 Hz**. Per-wheel velocity is low-pass filtered before the PI.

---

## Hardware

All peripherals operate on **3.3V logic** and connect to the STM32.

| # | Component | Qty | Notes |
|---|---|---|---|
| a | Pololu G2 24V14 | 1 | Dual DC motor driver |
| b | CZL601 load cell + HX711 | 4 each | 100 kg/cell; one per cargo-platform corner |
| c | Pololu QTR-8A | 1 | 8-channel analog reflectance / line sensor |
| d | OMRON E6B2 encoder | 2 | A/B channels; **500 PPR** (×4 = 2000 CPR) — see `config.h` |
| e | E18-D80NK proximity | 4 | NPN digital; one per side |
| f | BNO055 IMU | 1 | 9-DOF with onboard fusion; I2C1, 3.3V |
| g | VL53L0X TOF | 4 | I2C ranging; one per corner, behind the mux |
| h | TCA9548A I2C mux | 1 | Fans out the 4 same-address VL53L0X |
| i | INA219 | 2 | Bus-voltage sense (3S 0x40, 6S 0x41); no current sense |
| j | Arduino UNO + 3.5" TFT | 1 | USB status display off the Jetson; no touch |

---

## STM32 Pin Mapping

| Function | Pins |
|---|---|
| USART1 ↔ ESP32 | PB6 TX, PB7 RX |
| Encoder 1 (left), TIM2 | PA0 (CH1), PA1 (CH2) |
| Encoder 2 (right), TIM3 | PA6 (CH1), PA7 (CH2) |
| Motor 1 (left): PWM/curr/DIR/SLEEP | PA8 (TIM1_CH1), PA2 (ADC_IN2), PB0, PB1 |
| Motor 2 (right): PWM/curr/DIR/SLEEP | PA11 (TIM1_CH4), PA3 (ADC_IN3), PB2, PB3 |
| QTR-8A (analog) | PA4, PA5, PC0, PC1, PC2, PC3, PC4, PC5 |
| Proximity (EXTI) | PC6 Front, PC7 Rear, PC8 Left, PC9 Right |
| I2C1 bus | PB8 SCL, PB9 SDA (AF1; external 4.7 kΩ pull-ups) |
| HX711 (bit-banged) | PB10 shared SCK; PB12–PB15 DOUT (FL, FR, RL, RR) |

All four HX711s share SCK and are clocked in parallel. Proximity/HX711 facing and
corner assignments are logical labels in `config.h`.

**I2C1 bus devices** (all 3.3 V logic, shared PB8/PB9): the BNO055 IMU (0x28) and
two INA219 (0x40 = 3S, 0x41 = 6S, voltage-sense only) sit directly on the bus; a
TCA9548A mux (0x70) fans out four VL53L0X TOF sensors that all keep the default
0x29 — the mux exposes one channel at a time, so they never collide. The IMU and
INA219 are upstream of the mux and always reachable. No new STM32 pins are used.

---

## Firmware Structure

Two tiers, one module per folder, all headers reachable by bare name (the Makefile
adds every leaf module folder to the include path):

**`hal/` — bare-metal, talks to silicon only:**
`mcu` (clock/SysTick/IWDG/reset cause) · `uart` (USART1 + DMA RX ring/TX) ·
`pwm` (TIM1 + DIR/SLEEP) · `qenc` (TIM2/TIM3 quadrature) · `analog` (ADC+DMA scan) ·
`i2c` (I2C1 master) · `flash` (last-page calibration storage).

**`app/` — libraries built on the HAL:**
`proto` (frame codec + CRC + command/param dispatch) · `telemetry` (stream packing) ·
`log` (fault-log ring) · `safety` (mode/function state machine + E-STOP + caution +
heartbeat + cargo/current/TOF/battery monitors) · `control` (the per-tick chain) ·
`nav` (standby/remote/line/traj navigators) · `ramp` · `odometry` · `pid` · `motors` ·
`encoders` · `imu` (BNO055) · `loadcells` (HX711) · `proximity` (EXTI) ·
`tof` (VL53L0X ×4 via TCA9548A mux) · `battery` (INA219 ×2 bus voltage).

Init order in `main.c`: `mcu → log → motors (SLEEP low) → encoders → proto → analog →
loadcells → imu → tof → battery → safety → nav → load QTR cal → odometry → ramp →
control → proximity → telemetry`, then the IWDG starts (TOF init is long but runs
before the watchdog). The main loop drains RX, runs the heartbeat watch and sensor
ticks (incl. round-robin TOF + battery), applies SLEEP, runs the 100 Hz control
cadence (`encoders → odometry → monitors → control`, where `monitors` includes the
TOF distance-band and 3S low-voltage checks), forwards logs, and emits due telemetry.

---

## Coding Guidelines

**Priority order: Reliability → Performance → Readability** (readability is a first-class
goal, not an afterthought).

- Avoid race conditions, UB, and edge cases by design.
- **STM32:** bare-metal C with CMSIS register macros (no magic addresses). Prefer
  hardware peripherals (DMA, hardware encoder timers, ADC scan) over software polling.
  Every tunable lives in `config.h`. Keep the `hal`/`app` split clean: the HAL never
  embeds policy; the app never pokes registers directly (except single-peripheral
  bit-bang/EXTI owners — `loadcells`, `proximity`).
- **ESP32:** PlatformIO + Arduino. Mirror protocol constants (incl. `PROTO_VERSION`)
  with the STM32 side.
- **Jetson / GUI:** the ROS bridge is the protocol authority; the GUI and other ROS
  consumers stay decoupled behind topics/services and the WebSocket frame format.
