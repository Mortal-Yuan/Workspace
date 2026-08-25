# Smart Car Test

ESP-IDF 5.4.4 bring-up project for an ESP32-S3 three-wheel omnidirectional smart car.

Current verified features:

- Three PWM motor channels with direction control.
- Three AB encoder inputs.
- Four-channel active-low infrared line sensor input.
- Four-sensor proportional line-following mode with lost-line recovery.
- Non-blocking HC-SR04 ranging and line-follow obstacle avoidance.
- USB Serial/JTAG command console.

See [电机与红外测试记录.md](电机与红外测试记录.md) for the confirmed motor and infrared bring-up results. See [项目开发与实验日志.md](项目开发与实验日志.md) for complete wiring, algorithm iterations, problems, solutions, and physical-test results. See [AGENTS.md](AGENTS.md) for agent handoff notes.

## Build

Open this directory as an ESP-IDF project in VS Code, select target `esp32s3`, then use the ESP-IDF Build and Flash commands. The equivalent terminal workflow is:

```powershell
idf.py set-target esp32s3
idf.py build
idf.py -p COM5 flash monitor
```

The serial port is machine-specific; the current board has been detected as `COM5`.

## Serial Commands

| Command | Action |
| --- | --- |
| `1` | Run motor A / right wheel only |
| `2` | Run motor B / rear wheel only |
| `3` | Run motor C / left wheel only |
| `BOOT` or `f` | Start infrared line following |
| `m` | Toggle the 10 Hz line-follow monitor |
| `w` | Move forward using the verified wheel combination |
| `s` | Move backward |
| `x` | Stop all motors |
| `r` | Run a short A/B/C motor test |
| `+` / `-` | Increase/decrease PWM speed |
| `c` | Clear encoder counts |
| `h` | Print command help |

The current `a` and `d` commands have not been calibrated for this three-wheel chassis and should not be treated as valid turn commands yet.

At startup all motors remain stopped. Telemetry prints the four infrared inputs, ultrasonic distance, and A/B/C encoder counts every 500 ms.

## Line Following

Place the black line below the center two sensors, then briefly press the board's `BOOT` button or send `f`. Send `x` at any time to leave line-following mode and stop all motors. Manual motor commands also leave line-following mode. Do not hold `BOOT` while pressing `RESET`, because that combination enters the ROM download mode instead of starting line following.

Telemetry displays infrared detections as `IR[L,LC,RC,R]`. A value of `1` means that physical sensor currently detects black; `0` means white. This is the normalized detection value, not the active-low GPIO level.

The controller assigns scaled weights `-6, -2, +2, +6` from left to right. It uses the verified forward combination on motors A and C, applies proportional correction according to the detected line position, and keeps rear motor B stopped.

The center pair `0110` and all-black `1111` patterns use a straight-line base speed of `320/1000`. Other nonzero patterns normally use a `220/1000` curve base speed and proportional correction, with each curve motor command capped at `360/1000`. A pattern visible only on one physical side of the array is treated as a sharp-edge candidate and is limited to a `170/1000` base speed and `280/1000` motor command.

The controller separately latches the turn direction. An opposite-direction reading must persist for three consecutive 20 ms cycles before it can change this latch; until confirmed, steering continues in the latched direction. When all four sensors read white (`0000`), the controller rotates in place at `280/1000` using the latched direction. This prevents one transient edge reading at a sharp corner from reversing the subsequent search.

### Real-Time Monitor

The line-follow monitor is enabled by default and prints at 10 Hz while `f` mode is active. Use `m` to turn it off or back on. Example:

```text
FOLLOW t=12450ms state=STRAIGHT IR[L,LC,RC,R]=0110 pattern=0x6 stable=8 active=2 error=0 control=0 last=-1 lock=-1 candidate=0:0 base=320 lost=0 distance=420mm obstacle=CLEAR cmd[A,B,C]=-320,0,-320 encoder[A,B,C]=-820,0,795
```

- `state`: `STRAIGHT`, `CURVE`, `SEARCH_LEFT`, or `SEARCH_RIGHT` while following.
- `IR`: normalized black-line detections in physical left-to-right order.
- `pattern`: hexadecimal form of the same four infrared bits.
- `stable`: number of consecutive 20 ms control cycles with the current pattern, capped at 255.
- `error`: current raw weighted line error; negative is left and positive is right.
- `control`: error actually used for motor correction after direction validation.
- `last`: last non-zero raw line error retained for diagnostics.
- `lock`: validated search direction (`-1` left, `1` right, `0` not initialized).
- `candidate`: pending opposite direction and its consecutive-sample count.
- `base`: current forward base speed; it is zero while the all-white search is active.
- `lost`: consecutive all-white control cycles.
- `cmd`: actual limited motor commands sent to A/B/C.
- `encoder`: cumulative A/B/C encoder counts.

## Obstacle Avoidance

Obstacle avoidance is active whenever line following is active. HC-SR04 measurements use GPIO interrupts, so Echo measurement does not block the 20 ms line controller. The sensor is triggered every 60 ms.

Two consecutive readings at or below 250 mm trigger avoidance. The car brakes for 250 ms, then rotates right in place using `A=190, B=0, C=-190`. Once two readings are at least 350 mm away (after a minimum 850 ms turn), line following resumes. A maximum 2400 ms turn prevents the avoidance state from becoming stuck when Echo readings are unavailable. Avoidance no longer overwrites the validated line-search direction.

## Sharp-Corner Status

The direction latch, three-cycle direction confirmation, and edge-speed limit handle the great majority of sharp corners well enough for the current task. A small number of unusual entry angles may still produce imperfect steering or search behavior. Further sharp-corner work is therefore an optional optimization, not a required fix or a blocker for subsequent features. The earlier unsuccessful test and the later broader test result are both recorded in [项目开发与实验日志.md](项目开发与实验日志.md).

Follow logs include `distance` and `obstacle`. The obstacle state is `CLEAR`, `BRAKE`, or `TURN_RIGHT`; transition events are also printed as `OBSTACLE ...` lines.

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
