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
The final 2026-08-27 line-following, obstacle-avoidance, display corrections,
validation results, and Git archive procedure are recorded in
[2026-08-27_巡线避障显示优化与代码封存报告.md](2026-08-27_巡线避障显示优化与代码封存报告.md).

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
| `BOOT` | Start the fixed entry maneuver, then line-follow + obstacle supervisor |
| `RESET` | Hardware-reset the controller and stop the car |
| `f` | Start/restart the fixed entry maneuver, then autonomous line following |
| `t` / `y` | Directly run the pure right/left three-wheel vector at 420 for 100 ms, then stop |
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
| `v` | Enable the status display while stopped; native USB then disconnects |
| `u` | Enable the CRC-protected USB-UART camera preview and switch to 460800 bit/s |
| `z` | Disable the preview and return the application UART to 115200 bit/s |
| `b` | From `IDLE`, start the standalone red/blue-target align/approach/capture test |

The current `a` and `d` commands are legacy, uncalibrated combinations and
must not be used as kiwi-drive strafe commands. Use only the bounded `q/e`
calibration tests for the new three-wheel lateral basis.

At MCU startup all motors remain stopped. After `BOOT` or `f`, the ultrasonic
startup check must first report clear space. The car then executes one fixed
entry maneuver: forward at 400 for 1074 ms (500 for the first 150 ms), stop for
150 ms, pure clockwise yaw at 420 for 200 ms, and stop for another 150 ms.
This is the current open-loop estimate for 20 cm followed by about 60 degrees;
the turn duration is three quarters of the preceding 267 ms/80-degree estimate. The
camera line controller starts only after the final stop. A near obstacle can
still preempt the entry maneuver. `STATUS startup=0..5` exposes wait, forward,
forward-settle, right-turn, turn-settle, and line-follow-ready phases.

Telemetry prints camera/line inputs, ultrasonic distance, startup phase, and
A/B/C encoder counts every 500 ms.

The standalone `b` mode is deliberately separate from line following and
obstacle avoidance. It always starts by searching left and then right for the
red ball. During that search, the first blue component to become stably
detected is remembered as the preferred delivery target, regardless of which
side of the image contains it. If two blue components become available in the
same frame, stable-frame history and confidence choose between them; subsequent
frames use continuity rather than screen side to retain the chosen target.
Search uses short fast yaw pulses with a stopped camera observation after every
small angular step. Red-ball alignment and post-capture blue-target search use
the same 80 ms turn / 200 ms stopped-observation cadence. If the preferred blue
destination is already visible, short stopped-frame kiwi lateral
pulses first place the ball and target on approximately the same viewing ray.
If the destination is not visible, it does not wait: yaw pulses align the red
ball with the calibrated clip axis and the car approaches at decreasing speed.
If the preferred target becomes confirmed during alignment or forward approach, the
current motion stops and the controller immediately restores simultaneous
ball/target route pre-alignment; reaching the clip is not required first.
After two stationary in-clip observations, a missing destination starts an
alternating pulsed left/stop/right target search. Once either target is found,
the car aligns it with the clip axis using repeated 80 ms lateral shifts and
200 ms stopped observations, then starts pushing with a 420-command 180 ms
breakaway pulse, continues at 300, and stops on the
first red-center-inside-blue-box observation for two more stationary
confirmations (three total) before
`BALL_DONE`. Policy anomalies enter a two-second stationary recovery rather
than latching ball-mode failsafe. Press `x` to leave the mode. This mode is not
yet chained after obstacle `FINISHED`.

## USB Camera Monitor

The main line-follow/avoidance firmware can stream its processed 80x60 camera
view and normal telemetry through the CP210x USB-UART connection. Start the PC
window from PowerShell:

```powershell
powershell -ExecutionPolicy Bypass -File .\tools\start_usb_camera_monitor.ps1 -Port COM3
```

The window does not start the motors. It shows the camera image at about 5 FPS,
current mode, obstacle and line-follow states, camera geometry, ultrasonic
distance, A/B/C commands, and encoders. Red pixels are those admitted by the
same adaptive black threshold used by the vehicle; the yellow rectangle is the
actual analysis ROI, and the green/blue markers are the accepted near/far line
centers. Red/blue target detection uses the complete frame independently of that line
ROI. An orange box marks an unconfirmed red-ball candidate and a magenta box plus
center marker identifies a confirmed red ball; cyan marks `left_target` and
green-cyan marks `right_target`, even when an object is outside the yellow
rectangle or occupies only a few decoded pixels. The state panel reports
separate `BALL`, `LEFT_TARGET`, and `RIGHT_TARGET` observations plus strong-color
seed counts and exact pixel boxes. The red `Emergency stop`
button sends `x` immediately. The green `Start ball and push to blue goal`
button sends `b`;
it is accepted only while the top-level mode is `IDLE`.

The target mask uses hysteresis segmentation. Red uses permissive R>=45,
R-max(G,B)>=8, and R/RGB>=380-permille pixels preserve weak edges, but a blob
cannot become a ball without at least four stricter seeds satisfying
R-max(G,B)>=40 and R/RGB>=420 permille. Two bounded local-majority passes may
add adjacent bright low-chroma pixels to repair a white specular hole; those
pixels cannot seed a blob and are excluded from mean-color checks. Repaired
blobs still need sufficient area, fill, roundness, seed ratio, average red
dominance, and image-edge clearance. Blue uses B>=70, B-max(R,G)>=16, and
B/RGB>=390 permille for weak support; strong seeds require dominance>=45 and
ratio>=430 permille. The live centered blue rectangle additionally requires
mean blue dominance>=55 and permits 350-permille short/long-side ratio, while
red retains 600. Normal candidates require three stable
frames. A calibrated 40 cm component between 2 and 4 permille is admitted only
when confidence is at least 900 and requires five stable frames. The decoder
runs the same seeded-component pipeline independently for red and blue, so one
high-confidence goal cannot replace the red-ball observation. Target telemetry
does not affect line-follow or obstacle arbitration; only the explicitly
started standalone `BALL` mode may turn it into a motor command.

The application UART normally remains at 115200 bit/s. The monitor sends `u`,
waits for a low-rate acknowledgement, and then switches both ends to 460800
bit/s. It sends a one-second heartbeat; closing the window sends `z`, while a
lost heartbeat makes the firmware return to 115200 automatically after three
seconds. Each binary image carries a CRC32, so a damaged frame is discarded
without corrupting subsequent status parsing. The camera itself remains on the
ESP32-S3 USB-host D-/D+ pins; the PC window uses the separate CP210x cable.
If the board is reset while the window is open, the monitor detects the missing
valid frames and automatically negotiates the connection again. Use RESET alone
for a normal reboot; holding BOOT while releasing RESET selects ROM download
mode, where the camera application cannot run.

Near and far markers follow connected-track order rather than raw image height.
The firmware seeds the visible track entry near the previous center (or vehicle
center on first acquisition), propagates an eight-neighbour geodesic distance
through the selected component, and averages the first/last path thirds. This
keeps a rearward hairpin's returning leg from being mistaken for the near entry
merely because both legs appear at the bottom of the image.

## Status Display

The display is an `LQ_TFT18SPIV33` with an ILI9163B-compatible controller. The
driver uses Longqiu's TFT18 initialization and a 162x132 landscape layout with
`D/C=GPIO38`, `SDI/MOSI=GPIO20`, `SCK=GPIO45`, `CS=GPIO3`, `RST=GPIO47`,
`VCC=3V3`, and common ground. It shows the top-level mode, four infrared bits,
ultrasonic distance, obstacle state, motor command, and the final
`READY/RUNNING/FINISHED/FAILSAFE` condition. Rendering runs in a low-priority
task and does not execute in the 20 ms motion-control loop.

GPIO20 is also native USB D+. On reset the firmware deliberately leaves the
SPI display pins inactive and holds RST/GPIO47 low, so USB remains available
and the screen cannot retain a stale RUNNING image. Send
`v` for a stationary display check, or start with BOOT/`f`; the firmware then
stops USB access and assigns GPIO20 to the display, so COM disappearing is
normal. Press RESET to stop and restore the USB-capable startup state. A panel
connected to GPIO20 can load the USB line before firmware starts; if the port
is absent, temporarily unplug only SDI from GPIO20, flash and reset normally,
then reconnect SDI. GPIO45/SCK should have a 10 kOhm pull-down and GPIO3/CS a
10 kOhm pull-up to 3.3 V because both are strapping-related pins.

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

The active controller retains the final 2026-08-25 line-following geometry from
commit `da7a8aa` (source introduced by `10c8414`) and adds selective drive-wheel
authority for the measured startup problem. Rear motor B remains stopped. With
normalized sensor bits `L,LC,RC,R`, the integer line error is:

```text
error = (-6*L - 2*LC + 2*RC + 6*R) / active_sensor_count
correction = clamp(error * 120, -300, 300)
A = -base + correction, B = 0, C = -base - correction
```

- Center pair `0110` and all-black `1111`: base speed 348. Runtime finish
  recognition is disabled, so `1111` remains an ordinary line pattern.
- Other patterns spanning both sides: base 276, proportionally limited so the
  larger A/C magnitude is at most 384.
- One-sided patterns: base 233, proportionally limited to 336.
- Single-sensor patterns receive an explicit inside-wheel command:
  all four single-sensor patterns keep the inside wheel at least `+100`
  (`0001/0010 -> A=+100`, `0100/1000 -> C=+100`). This applies only when it agrees with
  the direction validator; an unconfirmed opposite sample cannot bypass the
  locked direction.
- Establishing or changing the locked direction requires three consecutive
  samples. Until an opposite direction is confirmed, control retains at least
  the locked-direction error instead of entering line-loss recovery.
- All-white `0000` keeps the previous cruise command for up to 120 ms, stops
  until 150 ms, then searches in the most recently remembered direction
  (locked direction, camera steering, or discrete error; default left). Search
  uses the 9.1 A/C-opposed command at 211 with B stopped, including the normal
  drive-wheel assist. Alternating legs are 1.2, 2.4, 3.6, 4.8, 6.0 and 7.2
  seconds, then remain capped at 7.2 seconds. Reacquisition requires any three
  distinct decoded frames containing a line; their directions need not match.

The shared manual/line speed ceiling defaults to 400. There is no blanket
nonzero floor. Instead, a wheel whose calculated target reaches at least 200 is
treated as a drive wheel: when it enters that role or reverses, it receives a
500 command for 150 ms and then returns to the calculated target. The explicit
100 single-sensor inside command and other low-demand inside-wheel commands such
as 43 or 44 remain below the assist threshold, preserving tight turning. The
later equal-wheel, speed-only experiment is preserved as
`main/control/archive/line_follow_speed_only_2026-08-27.c.disabled` and is not
compiled. The current rectangular obstacle-bypass controller is unchanged.

### Real-Time Monitor

The line-follow monitor is enabled by default and publishes a non-blocking
status snapshot at 10 Hz while autonomous mode is active. Use `m` to turn it
off or back on. Example:

```text
STATUS t=12450ms mode=AUTO obstacle=1 progress=0 IR=0110 pattern=6 line=1 err=0/0 base=360 us=520/520 q=1 echo=0 wait=0 timeout=0 anomaly=0 cmd=-360,0,-360 button=0/0/1 encoder=-820,0,795 overrun=0 drops=0
```

- `line`: numeric `IDLE`, `STRAIGHT`, `CURVE`, `SEARCH_LEFT`, or `SEARCH_RIGHT`.
- `IR`: normalized black-line detections in physical left-to-right order.
- `pattern`: hexadecimal form of the same four infrared bits.
- `err`: raw weighted error and the error used after direction validation.
- `base`: current forward base speed; it is zero while the all-white search is active.
- `cmd`: actual limited motor commands sent to A/B/C.
- `encoder`: cumulative A/B/C encoder counts.
- `overrun` / `drops`: control deadline misses and discarded diagnostic messages.

## Obstacle Avoidance

The fixed-segment rectangular bypass state machine is enabled for bounded field
calibration. Lateral directions are physically confirmed, but the current
1030/1074/1097 ms segments still require ruler calibration.

```text
SENSOR_CHECK -> CLEAR -> first real Echo from 20 through 80 mm -> BRAKE
BRAKE -> LEFT_STRAFE -> SETTLE_FORWARD -> FORWARD_TIMED
-> SETTLE_RIGHT -> RIGHT_RAMP (full 1097 ms)
-> FINAL_FORWARD_525MS -> FINISHED (latched stop)
```

`SENSOR_CHECK` keeps all motors stopped until three consecutive valid far-Echo
or clean no-return observations are available. While line following in `CLEAR`,
the first real Echo pulse with a raw distance from 20 through 80 mm immediately
writes all motor commands to zero
and enters `BRAKE`; it does not wait for the median or jump filter to accept the
sudden near reading. Every other ultrasonic result—including clean distance
jumps, no Echo, `LOST`, `NO_RETURN`, Echo-high and malformed-edge diagnostics—
is diagnostic-only and cannot alter line following.

The active sequence is: brake, strafe left for 1030 ms, settle, move forward
for 1074 ms, settle, then strafe right for 1097 ms. The middle forward segment
is 30% longer than the preceding 826 ms setting (1073.8 ms rounded to 1074 ms);
other timings remain unchanged. The obstacle trigger is 80 mm, 20 mm farther than
the preceding 60 mm setting.
Left strafe retains the 380 steady / 500 start-boost body commands, and forward
retains 400/500. Right strafe instead starts at 300 and linearly rises to 380
over 400 ms. During the same interval a small counter-clockwise body-yaw command
fades from -60 to zero, changing the launch wheel command from
`+300/+460/-300` to about `+300/+510/-300`; the established steady output
remains `+300/+570/-300`. Its 1097 ms duration is unchanged. Once
`BRAKE` starts, line following remains suspended: black, white and `1111`
camera patterns cannot shorten the right strafe or alter the remaining motion.
After the full right strafe, the controller drives straight with the fixed
500/400 start/steady commands for 525 ms, then enters the latched `FINISHED`
stop. A new near obstacle or repeated ultrasonic uncertainty during these fixed
segments still enters the latched fail-safe.

The candidate left strafe uses all three wheels as
`A=-0.866S, B=-S, C=+0.866S`; right strafe is its exact inverse. This result is
derived from the verified forward/clockwise bases and the 120-degree chassis
geometry, then corrected with independently configurable 50% left and 50%
right yaw compensation, plus per-direction dead-zone floors. See
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
