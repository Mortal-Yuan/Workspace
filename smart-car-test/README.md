# Smart Car Test

ESP-IDF 5.4.4 bring-up project for an ESP32-S3 three-wheel omnidirectional smart car.

Current verified features:

- Three PWM motor channels with direction control.
- Three AB encoder inputs.
- Four-channel active-low infrared line sensor input.
- Four-sensor proportional line-following mode with lost-line recovery.
- Non-blocking HC-SR04 ranging with pulse-edge validation, median filtering,
  quality states, and fail-safe obstacle stopping.
- USB Serial/JTAG command console.

The firmware is organized as small modules under `main/app`, `main/control`,
`main/drivers`, `main/diagnostics`, `main/core`, and `main/platform`. The
top-level controller is the only motion arbiter, and `motor_driver_apply()` is
the only final motor write point. See
[固件模块化重构计划.md](固件模块化重构计划.md) for the dependency rules, safety
contracts, extension path, and test matrix.

See [电机与红外测试记录.md](电机与红外测试记录.md) for the confirmed motor and infrared bring-up results. See [项目开发与实验日志.md](项目开发与实验日志.md) for complete wiring, algorithm iterations, problems, solutions, and physical-test results. See [AGENTS.md](AGENTS.md) for agent handoff notes.

The modular refactor, rectangular obstacle-bypass design, lateral-motion test
data, dead-zone corrections, and current field-test parameters are summarized in
[2026-08-26_模块化重构与横移避障开发总结.md](2026-08-26_模块化重构与横移避障开发总结.md).

## Build

Open this directory as an ESP-IDF project in VS Code, select target `esp32s3`, then use the ESP-IDF Build and Flash commands. The equivalent terminal workflow is:

```powershell
idf.py set-target esp32s3
idf.py build
idf.py -p COM3 flash monitor
```

The serial port is machine-specific; the board was detected as `COM3` on
2026-08-26.

## Serial Commands

| Command | Action |
| --- | --- |
| `1` | Run motor A / right wheel only |
| `2` | Run motor B / rear wheel only |
| `3` | Run motor C / left wheel only |
| `BOOT` | Start line-follow + obstacle supervisor |
| `RESET` | Hardware-reset the controller and stop the car |
| `f` | Start line-follow + obstacle supervisor |
| `t` | Directly run A=+420/B=-420/C=-420 for 100 ms, then stop |
| `q` | Candidate left strafe at 380/1000 for 1000 ms, then stop |
| `e` | Candidate right strafe at 380/1000 for 1000 ms, then stop |
| `g` | Verified forward basis at 400/1000 for 1000 ms, then stop |
| `j` | Run rear motor B at -380/1000 for 1000 ms, then stop |
| `k` | Run rear motor B at +460/1000 for 1000 ms, then stop |
| `m` | Toggle the 10 Hz line-follow monitor |
| `w` | Move forward using the verified wheel combination |
| `s` | Move backward |
| `x` | Stop all motors |
| `r` | Run a short A/B/C motor test |
| `+` / `-` | Increase/decrease PWM speed |
| `c` | Clear encoder counts |
| `h` | Print command help |

The current `a` and `d` commands are legacy, uncalibrated combinations and
must not be used as kiwi-drive strafe commands. Use only the bounded `q/e`
calibration tests for the new three-wheel lateral basis.

At startup all motors remain stopped. Telemetry prints the four infrared inputs, ultrasonic distance, and A/B/C encoder counts every 500 ms.

## Line Following

Place the black line below the center two sensors, then briefly press the board's
`BOOT` button to start the combined line-follow and obstacle supervisor. `BOOT`
is start-only; pressing it again does not stop a running car. Press `RESET`
alone to hardware-reset the controller and return to the startup state with all
motor commands at zero. Serial `f` starts and `x` stops without resetting.
Manual motor commands also leave autonomous mode. Do not hold `BOOT` while
pressing `RESET`, because that combination enters ROM download mode.

The BOOT input now uses a 50 ms stable-level debounce, requires an 80 ms stable
release before another press, and has a 500 ms startup guard. A valid short
press during that startup guard is queued rather than discarded. If BOOT is
already held when the MCU starts, it must first be released and pressed again;
this prevents a held boot-strap key from starting the car unexpectedly. The
top-level controller has explicit `IDLE`, `AUTONOMOUS`, `MANUAL`, `SELF_TEST`,
and `FAULT` modes, so command handling cannot create contradictory run flags.

Every all-zero motor command now also disables the motor driver's enable/STBY
pin; the driver is enabled only after nonzero directions and PWM have been
written. For guaranteed motor disable during the interval before firmware runs,
or while the ESP32-S3 is in ROM download mode, add an external approximately
10 kOhm pull-down to the driver's enable/STBY input if the motor board does not
already provide one. Firmware cannot read RESET/EN as an ordinary stop button,
because pressing it resets the MCU itself.

Telemetry displays infrared detections as `IR[L,LC,RC,R]`. A value of `1` means that physical sensor currently detects black; `0` means white. This is the normalized detection value, not the active-low GPIO level.

The controller assigns scaled weights `-6, -2, +2, +6` from left to right. It uses the verified forward combination on motors A and C, applies proportional correction according to the detected line position, and keeps rear motor B stopped.

The center pair `0110` and all-black `1111` patterns use a straight-line base
speed of `420/1000`. Other nonzero patterns normally use a `310/1000` curve
base with commands capped at `500/1000`. A pattern visible only on one side uses
a `280/1000` base and `450/1000` cap. Nonzero curve-wheel commands are held at
or above `220/1000` after scaling so that the inside wheel does not fall into
the measured low-speed stall region.

To improve startup under load without raising steady cruising speed, motors A
and C receive a short open-loop startup boost. On initial motion, or after a
commanded direction reversal, any requested magnitude below `500/1000` is raised
to `500/1000` for 150 ms and then automatically returns to the requested line
command. A true zero command must persist for 120 ms before the next same-way
start is eligible for another boost. The boost is used only by line following
and lost-line search; manual motor tests bypass it. This is intentionally
encoder-free until the motor-induced encoder glitches have been corrected.

The controller separately latches the turn direction. An opposite-direction
reading must persist for three consecutive 20 ms cycles before it can change
this latch. When all four sensors read white (`0000`), it searches at
`360/1000` using the latched direction.

### Real-Time Monitor

The line-follow monitor is enabled by default and publishes a non-blocking
status snapshot at 10 Hz while autonomous mode is active. Use `m` to turn it
off or back on. Example:

```text
STATUS t=12450ms mode=AUTO obstacle=1 progress=0 IR=0110 pattern=6 line=1 err=0/0 base=420 boost=0,0 us=520/520 q=1 echo=0 wait=0 timeout=0 anomaly=0 cmd=-420,0,-420 button=0/0/1 encoder=-820,0,795 overrun=0 drops=0
```

- `line`: numeric `IDLE`, `STRAIGHT`, `CURVE`, `SEARCH_LEFT`, or `SEARCH_RIGHT` state.
- `IR`: normalized black-line detections in physical left-to-right order.
- `pattern`: hexadecimal form of the same four infrared bits.
- `err`: raw weighted error and the error used after direction validation.
- `base`: current forward base speed; it is zero while the all-white search is active.
- `cmd`: actual limited motor commands sent to A/B/C.
- `encoder`: cumulative A/B/C encoder counts.
- `overrun` / `drops`: control deadline misses and discarded diagnostic messages.

## Obstacle Avoidance

The full rectangular bypass state machine is enabled for bounded field
calibration. Lateral directions are physically confirmed; the 800 ms lateral
clearance and 2500 ms forward segment are still provisional rather than
ruler-calibrated 10 cm and 40 cm distances.

```text
SENSOR_CHECK -> CLEAR -> first real Echo <=200 mm -> BRAKE
BRAKE -> LEFT_EDGE -> LEFT_CLEARANCE -> SETTLE_FORWARD
-> FORWARD_BYPASS -> SETTLE_RIGHT -> RIGHT_LINE
-> LINE_CONFIRM -> CLEAR
```

`SENSOR_CHECK` keeps all motors stopped until three consecutive clear
observations are available. While line following in `CLEAR`, the first real
Echo pulse with a raw distance from 20 through 200 mm immediately writes all
motor commands to zero and enters `BRAKE`; it does not wait for the median or
jump filter to accept the sudden near reading. `WAIT_CLEAR` remains only as the
fail-safe recovery state for an uncertain ultrasonic observation while normal
line following is active.

The active sequence is: brake, strafe left until three clear ultrasonic
observations, strafe left for an additional provisional 800 ms, settle, move
forward for a provisional 2500 ms, settle, then strafe right. The first infrared
line detection stops the motors immediately and the line must remain present
for five control cycles before line following resumes. Every search has a
timeout and repeated sensor uncertainty enters a latched fail-safe stop.

The candidate left strafe uses all three wheels as
`A=-0.866S, B=-S, C=+0.866S`; right strafe is its exact inverse. This result is
derived from the verified forward/clockwise bases and the 120-degree chassis
geometry, then corrected from the bounded `q/e` physical tests with 20% yaw
compensation and per-direction dead-zone floors. See
[三轮横移标定与矩形避障实施计划.md](三轮横移标定与矩形避障实施计划.md) for the acceptance
criteria and measurement table.

The first 2026-08-26 post-flash UART diagnostic reported Echo high,
approximately 8113--8116 mm invalid raw pulse widths, and repeated timeouts.
The ultrasonic heads were resting against the tabletop during that test. After
the sensor was positioned normally, a 10-second retest produced only `VALID`
samples at 231--263 mm, idle Echo low, and zero timeouts. The earlier result was
therefore caused by the blocked/near-field acoustic setup rather than a GPIO13
wiring fault. This car currently powers the module from 3.3 V, so its Echo is
connected directly; a divider is required if the module is later powered from
5 V.

## Sharp-Corner Status

The direction latch, three-cycle direction confirmation, and edge-speed limit handle the great majority of sharp corners well enough for the current task. A small number of unusual entry angles may still produce imperfect steering or search behavior. Further sharp-corner work is therefore an optional optimization, not a required fix or a blocker for subsequent features. The earlier unsuccessful test and the later broader test result are both recorded in [项目开发与实验日志.md](项目开发与实验日志.md).

Follow logs include raw/filtered distance, ultrasonic quality, obstacle state,
and the clear counter. Transitions are printed as `OBSTACLE ...` lines with a
reason and the current clear count.

## Control References

- [Pololu QTRSensors `readLineBlack()` documentation](https://pololu.github.io/qtr-sensors-arduino/class_q_t_r_sensors.html): weighted-average line position and retention of the last observed side when the line is lost.
- [Pololu 3pi Advanced Line Following with PID Control](https://www.pololu.com/docs/0J21/7.c): proportional position error, derivative of successive errors, and differential motor correction.
- [WPILib Introduction to PID](https://docs.wpilib.org/en/stable/docs/software/advanced-controls/introduction/introduction-to-pid.html): the derivative term acts on error rate and can amplify sensor noise; this rollback intentionally uses only the proportional part while hardware and sensor behavior are isolated.
- [ROS 2 Nav2 recovery behavior tree](https://github.com/ros-navigation/navigation2/blob/main/nav2_bt_navigator/behavior_trees/navigate_to_pose_w_replanning_and_recovery.xml): recovery actions are tried in a round-robin sequence that includes both `Spin` and `BackUp`, rather than repeating rotation at one position.
- Aydin Gullu and Hilmi Kuscu, [Evaluation of Search Strategy for Autonomous Rescue Mobile Robot](https://doi.org/10.1109/ICAI52893.2021.9639868), IEEE, 2021: motivates systematic search trajectories that cover an expanding area instead of a fixed-point maneuver.
- Vishnu Agarwal, [Line Following at Sharp Turns](https://doi.org/10.1007/978-1-4842-9706-3_26), Apress, 2023: sharp turns require explicit handling beyond ordinary continuous line correction.
- [Pololu 3pi Advanced Line Following with PID Control](https://www.pololu.com/docs/0J21/7.c): continuous position and derivative feedback is appropriate for normal tracking, while the documented controller still requires separate tuning for large heading errors.
- [Pololu QTRSensors `readLineBlack()`](https://pololu.github.io/qtr-sensors-arduino/class_q_t_r_sensors.html): weighted-average position is monotonic and retains the last direction when the line is temporarily absent; this supports hysteresis and reacquisition validation.
- D. Vijendra Babu et al., [Line follower robot & obstacle detection using PID controller](https://doi.org/10.1063/5.0024760), AIP Conference Proceedings, 2020: combines line-feedback control and obstacle detection as supervisory behaviors.
