# Smart Car Project Context

This file is the handoff summary for agents working in `smart-car-test`.

## Project Scope

- Target board: ESP32-S3-DevKitC-1 / ESP32-S3-WROOM-2.
- Framework: ESP-IDF 5.4.4.
- Course platform: three-wheel omnidirectional smart car.
- Current phase: replacing the four-channel infrared sensor with a USB UVC
  camera, then calibrating camera line following and ultrasonic obstacle bypass.
- Course direction from the lecture: line following and obstacle avoidance first, with camera/ball interaction and optional extensions in later phases.

The detailed, experimentally confirmed motor and infrared mapping is in [电机与红外测试记录.md](电机与红外测试记录.md). The complete development history, wiring summary, algorithm experiments, failures, and latest physical-test conclusions are in [项目开发与实验日志.md](项目开发与实验日志.md). The final 2026-09-01 camera, obstacle, finish, ball, wiring, and flashed-artifact handoff is in [2026-09-01_摄像头巡线避障与红球识别综合开发总结.md](2026-09-01_摄像头巡线避障与红球识别综合开发总结.md). Treat those documents and the current source as authoritative when older notes disagree.

## Local Reference Material

The workspace contains a local `参考文件` directory. Its raw PDFs, RAR archives, extracted images, OCR text, and temporary analysis directories are intentionally not committed.

Important source files inspected locally:

- `2026夏 - 电子技术课程设计绪论课讲稿.pdf`
- `esp32-s3-wroom-2_datasheet_cn.pdf`
- `vscode-esp32s3测试程序.rar`
- `arduino-esp32s3测试程序.rar`
- `智能小车项目_传感器型号与接口汇总.md`
- `智能小车项目_电机红外超声波接线规划.md`

The lecture identifies these course devices:

| Device | Model or interface noted in the material |
| --- | --- |
| Main controller | ESP32-S3-DevKitC-1 / ESP32-S3 |
| Motor driver | DA24 / TB6615 family, three motor outputs M1/M2/M3 |
| DC gear motors | LQ25R8V6 with AB Hall encoders |
| Line sensor | LQ-IR4CHV6 four-channel infrared module |
| Ultrasonic sensor | HC-SR04 |
| Inertial sensor | MPU6500 |
| Optional temperature/humidity sensor | DHT11 |
| Servo | MG90S |
| Bluetooth module | BT08B |
| USB serial module | CH340 |
| Display | TFT IPS |
| Course camera/audio module | ESP-Claw module; exact connector pinout was not provided |

The supplied VS Code archive contains an ESP-IDF LED blink project, and the Arduino archive contains only an LED sketch. Neither archive provides the three-motor pin mapping or omnidirectional kinematics.

## Confirmed Hardware State

- Motor A is the right wheel.
- Motor B is the rear wheel.
- Motor C is the left wheel.
- All three motors and all three encoder pairs have produced valid activity.
- Confirmed forward command: `A=-speed, B=0, C=-speed`.
- Confirmed backward command is the exact inverse.
- The four-channel infrared module has been retired. Its VCC/GND/OUT wires may
  be removed; GPIO1/2/4 are free and GPIO5 is reassigned to display MOSI.
- The verified UVC camera is VID `0x349c`, PID `0x3307`, wired as
  `D-=GPIO19`, `D+=GPIO20`, `5V`, and common `GND`. It supports the selected
  MJPEG 640x480 @ 15 fps profile. The camera was physically remounted upright
  on 2026-08-28, so preview and vision now use the native image orientation.
- HC-SR04 is assigned `Trig=GPIO6`, `Echo=GPIO13`. Non-blocking GPIO-edge timing feeds obstacle avoidance while line following. Ranging accuracy and the physical Echo level interface have not yet been formally validated.
- The current UART0 diagnostic uses `TX=GPIO43` and `RX=GPIO44`
  at 115200 bit/s, so those pins are not available for MPU6500 I2C.

Important allocation changes from early notes:

- GPIO41/GPIO42 are now used by motor C encoder E3A/E3B, so they are not available for the earlier proposed MPU6500 I2C connection.
- GPIO14 is now motor B `BIN1`, so the earlier optional DHT11 GPIO14 proposal is invalid.
- Early two-wheel documents label A/B as left/right. Actual tests supersede that: A=right, B=rear, C=left.

## Source Behavior

The firmware is modular. `main/app_main.c` is the entry point;
`main/app/app_controller.c` owns top-level mode and final motion arbitration;
hardware access is isolated under `main/drivers` and `main/platform`; pure
control policies are under `main/control`. See `固件模块化重构计划.md` for the
dependency and extension contracts.

- Motors are stopped at startup.
- Manual motion and the adjustable line-speed ceiling default to `400/1000`.
- Each `BOOT`/`f` autonomous start first waits for the obstacle supervisor's
  three-sample clear authorization, then runs one fixed open-loop entry
  maneuver before camera steering is allowed: forward for 1074 ms at 400
  (500 during the first 150 ms), zero for 150 ms, pure clockwise yaw at 420
  for 200 ms, and zero for 150 ms. The wheel vectors are respectively
  `A=-speed,B=0,C=-speed` and `A=+420,B=-420,C=-420`. These are the current
  time-based estimates for 20 cm and about 60 degrees; the turn duration is
  three quarters of the preceding 267 ms estimate. No trustworthy gyro or
  motor-on encoder angle is available. Near-obstacle arbitration, faults,
  `x`, and manual commands can still preempt the sequence. At completion the
  line controller is reset from the latest camera frame so no pre-turn
  direction memory survives. UART `STATUS startup=0..5` exposes the phase.
  The active line controller retains the final 2026-08-25 geometry from commit
  `da7a8aa` (source introduced by `10c8414`). It uses normalized
  `L/LC/RC/R` weights `-6/-2/+2/+6`, divides by the active-sensor count, and
  applies `correction=clamp(error*120,-300,300)` to
  `A=-base+correction, B=0, C=-base-correction`.
- Camera line bases/limits are straight 348, ordinary curve 276/384,
  one-sided edge 233/336, and no-line search 211. Normal driving bases, curve
  and edge limits, proportional gain, and maximum correction are 20% above the
  preceding 290/230/194, 320/280, 100, and 250 settings (rounded to integers).
  This scales normal steering commands without flattening their differential;
  If line loss
  immediately follows reliable cruise motion, the controller keeps the last
  unassisted cruise command for 120 ms to cross the camera's near blind spot,
  commands zero for the remaining 30 ms, then begins the 2026-09-01 search.
  Startup without line/motion history commands zero for the full 150 ms. The
  first search direction comes from the locked direction, last camera steering,
  or last discrete error, falling back to left only when no history exists.
  Search uses the 9.1 A/C-opposed, B-zero vector at 211 and therefore passes
  through the normal 500-command drive assist. Consecutive search legs alternate
  and last 1200, 2400, 3600, 4800, 6000, and 7200 ms; later legs remain capped
  at 7200 ms. Search has no total timeout. A candidate stops rotation immediately
  and any three distinct decoded frames containing a line complete reacquisition;
  their left/center/right directions do not have to agree. Initial direction
  locking takes three samples. Once locked, an opposite observation is held to
  the previous direction until the opposite direction itself is confirmed three
  times; it does not immediately enter line-loss recovery.
- There is no blanket nonzero floor. A wheel with a calculated absolute target
  of at least 200 gets a 500/1000, 150 ms assist only when it becomes a drive
  wheel or reverses. All four single-sensor patterns keep the inside wheel at
  least `+100`: `0001/0010: A=+100` and `0100/1000: C=+100`; this override
  requires agreement with direction validation. The 100 command and other
  43/44 inside targets are below the assist threshold.
  Suspend/resume resets assist state so obstacle recovery can start cleanly.
- The later equal-wheel, speed-only experiment is preserved as
  `main/control/archive/line_follow_speed_only_2026-08-27.c.disabled` and is
  excluded from CMake. The active obstacle supervisor uses the fixed-distance
  calibration sequence described below and is independent of line control.
- `1`, `2`, `3` run A/right, B/rear, C/left individually.
- `w`, `s`, `x` are confirmed forward, backward, and stop.
- A brief press of the board's `BOOT` button (GPIO0, active low) starts the
  combined line-follow/obstacle supervisor and is ignored while it is already
  active. Pressing the hardware `RESET`/EN button alone resets the MCU; startup
  immediately writes zero motor commands and disables motor-driver EN/STBY, so
  RESET is the physical stop control.
  Serial `f` starts and `x` stops; any manual motor command exits autonomous
  mode. Holding `BOOT` during reset still enters the ESP32-S3 ROM download mode.
- BOOT has a 50 ms stable-level debounce, an 80 ms release-to-rearm time, and a
  500 ms startup guard. A valid press during the guard is queued. BOOT held at
  startup is not armed until it is released, preventing unintended startup.
  Top-level operation uses explicit
  `IDLE/AUTONOMOUS/MANUAL/SELF_TEST/FAULT` modes; BOOT cannot be blocked by a
  stale combination of run flags.
  All-zero motor commands disable EN/STBY, and nonzero commands enable it only
  after directions/PWM are written. An external roughly 10 kOhm EN/STBY
  pull-down is still recommended for guaranteed disable before firmware starts
  and in ROM download mode; software cannot guarantee that reset interval.
- `m` toggles the real-time line monitor, which defaults to 10 Hz while following.
- `r` runs a short A/B/C sequence.
- `t` and `y` run bounded 100 ms three-wheel pure-yaw tests at 420 for right
  (`+420/-420/-420`) and left (`-420/+420/+420`) respectively.
- `+` and `-` adjust speed.
- USB Host receives 640x480 MJPEG at 15 fps. The newest source has been restored
  to `JPEG_IMAGE_SCALE_1_8`, decoding an 80x60 RGB888 frame. The full decoded
  frame reaches ball vision, while its lower 80x30 half
  reaches the line-vision entry point. The line task still applies its existing
  central ROI after that crop.
  The active native-view ROI uses the central horizontal span
  `x=25%..75%` while retaining `y=60%..93%`. Horizontal coordinates, component
  width, and component area use the calibrated 1x logical scale. The dark
  histogram percentile is 2% for the restored central-width background. The
  adaptive black threshold uses the 9.1 formula
  `min(Otsu + contrast/8, 120)`. The absolute 120/255 ceiling prevents a
  relatively dark grey shadow from becoming black merely because it forms
  Otsu's darker class. This is followed by true 8-neighbour two-dimensional connected-component
  labeling. Every non-empty connected component is eligible. Without a stable
  line history, the component with the largest pixel area is selected. After
  three consecutive ordinary-line frames, the last reliable near position and
  steering direction softly rank later components by
  `area/(250 + position_error + steering_error)`. There is no minimum area,
  bottom-touch, height, width, initial-center, or history-jump gate: a large
  jump loses score but is never rejected, and a sole connected component is
  still accepted. Disconnected islands do not merge.
- Each accepted component produces a near center, a far center, and
  `heading=far-near` using the 9.1 image-row rule: the lowest third of the
  component's Y span is near and the highest third is far. This is not a
  connected-path/geodesic ordering, so a returning hairpin leg low in the image
  can influence the near point. Continuous
  steering normally uses `near + heading*0.5`, so the visible line beginning
  about 10 cm ahead is used as a fixed lookahead rather than pretending to
  measure under the wheels. On a sharp hairpin where near and far lie on
  opposite image sides (`abs(near)>=160`, `abs(heading)>=300`), the heading
  gain falls to 0.1. This prevents the far term from cancelling the near error;
  the captured `near=+250, far=-160` case changes from about `steer=+45` to
  `+209`. Candidate eligibility remains pure connected-component selection.
  When a newly established camera turn reaches 180 permille, the controller
  holds the previous proven cruise trajectory for 180 ms before applying the
  turn. The turn latch releases below 80 permille so it cannot restart on every
  frame.
  The line controller scales this steering continuously to its +/-300 maximum
  correction at +/-600 permille; virtual `L/LC/RC/R` remains available for
  pattern state, direction validation, obstacle recovery, and diagnostics.
- Position and steering history participate only in soft candidate ranking,
  never in candidate acceptance. The implemented T-finish classifier requires a
  connected component at least 800 permille wide and 200 permille in area, but
  its runtime gate is currently kept disabled. T-like shapes therefore remain
  ordinary connected lines and cannot finish autonomy. Three accepted
  normal-line frames separately arm only the soft history ranking.
  Vision scratch buffers live in one
  PSRAM workspace allocated at camera initialization, not on the decode-task
  stack and not per frame.
- Line following actively steers with A/C differential correction and searches
  after line loss. BOOT/`f` requires a fresh decoded camera result but no longer
  requires a currently accepted connected line. A fresh `CAM=0000` result,
  including low contrast or no connected component, starts autonomy and enters
  the alternating search. A camera result older than 350 ms is a safety failure
  while autonomy is waiting to follow or actively following the line: it enters
  `FAULT` and disables the motors in that control cycle. Once a near obstacle
  enters `BRAKE`, the remaining bypass is deliberately fixed-motion and no
  longer consumes line input, so camera freshness no longer gates that run.
- Telemetry prints the virtual pattern plus camera freshness, decoded sequence,
  near center (`pos`), far center, heading, steering, width, connected-component
  count/height/area (`comp=count/height/area`), black fraction, threshold,
  contrast, drops, errors, ultrasonic distance, and A/B/C encoder counts. In the virtual pattern, `1` means the
  corresponding camera-derived lane region sees black.
- The same decoded 80x60 RGB888 frame feeds a full-frame red/blue-target
  detector in `main/control/camera_ball_vision.c`; the yellow line ROI does not
  restrict it. It does not influence line following or obstacle avoidance;
  only the separately commanded standalone `BALL` mode may consume it for
  motion. Hysteresis color
  segmentation admits weak red edges at R>=45, R-max(G,B)>=8, and R/RGB>=380
  permille. Blue uses B>=70, B-max(R,G)>=16, and B/RGB>=390 permille for weak
  support, with strong blue seeds at dominance>=45 and ratio>=430 permille.
  Every accepted blob must contain at least four strong seeds; strong red uses
  R-max(G,B)>=40 and R/RGB>=420 permille. Strong seeds must comprise at least
  80 permille of its colored pixels. Two local-majority passes can add adjacent
  bright low-chroma pixels (maximum channel >=160, chroma <=48) to repair small
  white specular holes within one color; these support pixels cannot seed a component and are
  excluded from its mean RGB. Eight-connected repaired red blobs need mean R
  at least 24 above mean max(G,B); blue blobs need mean B at least 55 above
  max(R,G). Both normally need 4 permille frame area (about 20 pixels at 80x60),
  350 permille bounding-box fill, and a two-pixel image-edge margin. Red retains
  600 permille short/long-side roundness; the calibrated blue rectangle uses
  350 permille. Highlight repair remains
  strictly bounded to two decoded passes;
  scaling it to 16 caused multi-second frames and background growth during the
  first live trial. Passing blobs outrank rejected diagnostics, then a weighted
  color/seed/area/shape confidence chooses among peers. A 2-to-3-permille
  component may pass only at confidence >=900 and then requires five frames;
  ordinary components require three. Tracking uses 200 permille X/Y tolerance.
  Each decoded frame is now analyzed twice with color-specific masks. UART
  `BALL` is the independent red-ball observation and `GOAL` is the independent
  blue-destination observation; each reports color/candidate/detected/stable
  count. Bounding boxes are normalized 0..1000. The unchanged 80x60 RGB332
  preview draws orange/magenta for the red ball and cyan/blue for the goal,
  including outside the line ROI. Command `b` starts the separately tested
  collect-and-push state machine only from `IDLE`: it laterally pre-aligns red
  and blue bearings with 90 ms kiwi pulses, aligns and approaches the red ball,
  verifies contact, steers a 210-command push from the blue center, and stops
  immediately when the red center enters the blue box plus a 60-permille
  margin. A second stationary overlap frame latches completion. This remains
  separate from the main course sequence.
- Camera vision analyzes the native view's central near-track window
  (`x=25%..75%`, `y=60%..93%`). Pixels outside that horizontal window are not
  included in the histogram, connected components, or line tracking. The
  current geometric-center calibration uses a
  -165 permille installation offset. After reflashing, a 20-second stationary
  run reported `pos=-41..-39`, `steer=-54..-53`, always `0110`, with no camera
  drop or decode error. The weighted centroid retains fractional-column
  precision before normalization.
- The cropped window is divided into five equal steering bands at -600, -200,
  +200, and +600 permille. The wider center band avoids one-pixel quantization
  jitter toggling a physically centered car between straight and curve.
- Follow-mode logs publish the raw weighted error, direction-validated control
  error, selected base speed, pattern state, and final A/B/C command.
- HC-SR04 samples have `VALID/OUTLIER/LOST/INVALID/NO_RETURN` quality, actual
  Echo edge-level validation, a three-sample median, and jump confirmation.
  Ranging now uses a nominal 45 ms deadline and a 70 ms trigger period. A
  completed pulse above 4000 mm is reported as `NO_RETURN` (`q=5`). Completed
  no-return pulses, low-Echo timeouts, and clean far-distance `OUTLIER` jumps
  are treated as normal open space both during startup authorization and after
  entering `CLEAR`. In active line following, the 20--80 mm raw-distance
  test is the only ultrasonic condition allowed to interrupt `CLEAR`; even an
  Echo-high or malformed-edge diagnostic does not alter the line policy.
  Automatic bypass
  is enabled for bounded field calibration. The first raw Echo from 20 through
  80 mm stops immediately, then the active sequence is `BRAKE -> LEFT_STRAFE
  -> SETTLE_FORWARD -> FORWARD_TIMED -> SETTLE_RIGHT -> RIGHT_RAMP ->
  FINAL_FORWARD_525MS -> FINISHED`. The experimental camera-heading alignment state and all its
  configuration were removed after the live 2026-09-01 test turned once and
  ended in latched `FAILSAFE`. `BRAKE` now transitions directly to left strafe
  after 150 ms. Because encoder interference is not yet filtered, the distance segments
  are scaled open-loop times: left 1030 ms, forward 1074 ms, and right 1097 ms.
  Left retains its 380/500 steady/start-boost body commands and forward remains
  at 400/500 commands. Right strafe now starts at the lowest effective body
  command 300 and linearly accelerates to 380 over 400 ms instead of applying
  the old 500-command launch boost. A right-launch-only counter-clockwise body
  command now fades from -60 to zero over the same 400 ms. This raises the
  dead-zone-clamped launch vector from `+300/+460/-300` to
  `+300/+510/-300`, reaches about `+300/+540/-300` halfway, and leaves the
  proven `+300/+570/-300` steady vector unchanged. The 1097 ms maximum approximately preserves
  the previous motion-command integral after introducing the ramp. The obstacle
  trigger is now 80 mm, 20 mm farther than the preceding 60 mm setting. All segments still
  require ruler calibration.
  The right segment now always completes its full 1097 ms. Line samples are
  ignored after `BRAKE`; there is no early black-line stop, `LINE_CONFIRM`, or
  return to line following. The following 525 ms is a fixed straight override
  using the same 500/400 start/steady forward commands, after which the car
  enters the latched `FINISHED` stop state.
  Near-obstacle and repeated ultrasonic uncertainty faults remain fail-safe
  stops. `1111` is always treated as an ordinary line pattern and no longer
  triggers finish.
- Ultrasonic authorization requires three consecutive obstacle-free
  observations; either a valid far Echo or clean low-Echo no-return/timeout
  counts, matching the mostly open competition course. Echo-high, malformed,
  or electrically uncertain samples never authorize startup. After entering
  normal `CLEAR` line following,
  every ultrasonic result other than a raw Echo from 20 through 80 mm is
  diagnostic-only and cannot stop or modify line following. Fixed-distance
  bypass segments do not interpret no-Echo as an obstacle-edge completion
  signal; repeated uncertainty during an already active bypass may still enter
  the maneuver fail-safe.
- The first 2026-08-26 post-flash UART diagnostic showed GPIO13 Echo high, raw
  8113--8116 mm invalid pulses, and repeated timeouts while the ultrasonic heads
  were resting against the tabletop. After normal positioning, a 10-second
  retest produced only `VALID` 231--263 mm samples, Echo low at idle, and zero
  timeouts. The failure was the blocked/near-field acoustic setup, not a GPIO13
  wiring fault. The module is currently powered from 3.3 V and Echo is direct;
  use a divider if changing it to 5 V supply.
- The current `a` and `d` implementations are inherited from the old two-wheel
  assumption and are not valid three-wheel strafe commands. A centralized kiwi
  inverse-kinematics module now derives candidate left strafe as
  `A=-0.866S, B=-S, C=+0.866S` before empirical dead-zone and yaw correction.
  B positive has a 460 minimum and pure lateral A/C have a 300 minimum. Yaw
  compensation remains direction-specific: left is currently 55% clockwise to
  remove slight counter-clockwise drift, while right remains 50%.
  At the 380 steady lateral command this produces
  `A/B/C=-300/-589/+300` left and `+300/+570/-300` right; the 500 start
  command produces `-300/-775/+300` and `+300/+750/-300`. Automatic bypass is enabled for bounded field
  calibration after `q/e` confirmed physical direction. `q/e` run for 1000 ms; `g` runs the verified
  forward basis at 400/1000 for 1000 ms for ruler calibration.

## Build and Flash

## Status Display (LQ_TFT18SPIV33 / ILI9163B)

- The identified 7-pin module is `LQ_TFT18SPIV33`. The active driver follows
  Longqiu's ILI9163B TFT18 power, gamma, frame-rate, VCOM, address and MADCTL
  initialization, with a 162x132 landscape drawing area in SPI mode 0:
  `D/C=GPIO38`, `SDI/MOSI=GPIO5`, `SCK=GPIO45`, `CS=GPIO3`, `RST=GPIO47`,
  `VCC=3V3`, and common `GND`. The old provisional ST7789 240x240 driver is
  invalid for this module and has been removed.
- GPIO19/20 are permanently reserved for camera USB D-/D+. The former display
  `SDI--GPIO20` jumper must be moved to GPIO5; leaving both attached is an
  electrical conflict. Native USB Serial/JTAG diagnostics are disabled and all
  flashing/commands use the CP210x UART on GPIO43/44.
- GPIO47 now performs the module-required 50 ms low / 50 ms high hardware
  reset before register initialization. GPIO45 is a strapping pin; keep SCK
  low at reset with an external 10 kOhm
  pull-down. GPIO3 is also a strapping/JTAG-selection pin; keep CS inactive
  high with an external 10 kOhm pull-up to 3.3 V.
- RESET remains the physical stop/recovery control. Do not hold BOOT while
  pressing RESET except when intentionally entering the ROM downloader.

User workflow requirement recorded on 2026-08-27: after every firmware program
change, automatically run the host regression, build the ESP-IDF image,
rediscover the connected board port, flash the new image, and perform a short
no-command startup check. Do not wait for a separate flash request. If the board
is disconnected or flashing is unsafe/blocked, report that condition explicitly.
Documentation-only edits do not require reflashing an unchanged image.

Normal ESP-IDF workflow:

```powershell
idf.py set-target esp32s3
idf.py build
idf.py -p COM3 flash monitor
```

On 2026-08-26 the board enumerated as CP210x `COM3`; always rediscover the port
instead of assuming it. Standard flashing has previously timed out while
uploading the stub. The reliable fallback is:

```powershell
python -m esptool --chip esp32s3 -p COM3 -b 115200 `
  --before default_reset --after hard_reset --no-stub write_flash `
  --flash_mode dout --flash_freq 80m --flash_size 32MB `
  0x0 build/bootloader/bootloader.bin `
  0x8000 build/partition_table/partition-table.bin `
  0x10000 build/smart-car-test.bin
```

These fallback flash parameters match the connected WROOM-2 board verified by
`esptool flash_id` (32 MB OPI flash). Recheck `build/flash_args` after changing
the target or sdkconfig; forcing the former 2 MB value makes the 4 MB factory
partition invalid and causes a boot loop.

On 2026-09-01 the first device enumerated as `Silabser0` CP210x `COM3` was an
unrelated ESP32-S2, so the S3 image was rejected before any write. The car was
then reconnected on the same port and positively identified as ESP32-S3 rev0.2,
16 MB PSRAM, MAC `9c:cc:01:fb:8f:9c`. The final -383 calibration image was
written with ROM `no-stub` at 115200 baud and all three Flash hashes verified.
The no-command check remained `IDLE`, A/B/C=0, encoders 0/0/0, camera fresh,
and no decode errors. Continue to verify chip identity rather than trusting the
COM number alone.

After the slower per-frame alignment image was flashed on 2026-09-01, the
no-command state again remained `IDLE` with A/B/C=0, a valid 234--237 mm
ultrasonic result, and fresh camera frames. However, after an explicit encoder
clear, all three counts advanced by tens of thousands per 500 ms while the
motor command stayed zero. This is electrically impossible as real wheel
motion and confirms that the unpulled, unfiltered encoder inputs are currently
noisy; do not use these counts for motion control until that input problem is
fixed. The image size is 0x59360 bytes and its SHA-256 is
`C2B5A784DB030274AD1ECE5C45C5890917771315D7F2976F5B46C52311319724`.

The subsequent 100 ms non-overlapping alignment-pulse image was flashed to the
same verified S3 on 2026-09-01. Host tests, the ESP-IDF build, and all three
flash hashes passed. Its no-command check stayed `IDLE`, A/B/C=0, encoders
0/0/0, camera fresh with zero decode errors, and ultrasonic quality `VALID`.
The image is 0x59440 bytes with SHA-256
`5669A177357A433BE1012EB1AB68AD09154501A019483D70081326A66157656A`.

The next field-requested image changes only the bounded alignment pulse from
100 ms to 30 ms. It passed host tests, the ESP-IDF build, chip verification,
flash hash checks, and a no-command check (`IDLE`, A/B/C=0, encoders 0/0/0,
camera fresh/error-free, ultrasonic `VALID`). Its size remains 0x59440 bytes;
SHA-256 is
`54A0DF521FD8D153F9E9CB418B7EC0A8359930FDCFA819FD93BA71113AF016BB`.

The live 30 ms test then made one alignment turn and ended in latched
`FAILSAFE` (`obstacle=12`, zero motor command). At the user's request, the
active 2026-09-01 image fully removes the alignment state, transitions,
configuration, runtime fields, camera-snapshot interface, and host-test helper.
After the 150 ms brake it enters left strafe directly. Host tests and the full
ESP-IDF build pass. The verified ESP32-S3 rev0.2 at MAC
`9c:cc:01:fb:8f:9c` accepted all three images with valid Flash hashes. The
final application is 0x59150 bytes with SHA-256
`0DB9BE4A7E5116F6638AFCCAD729751CDB45E44297A5053D649E4C5E454009A9`.
Its no-command check stayed `IDLE`, `obstacle=0`, A/B/C=0, encoders 0/0/0,
camera fresh/error-free, and ultrasonic `VALID` at about 329--337 mm. A
separate unexpected `POWERON` reset observed during live wiring still needs
power and connection investigation; do not attribute it to the removed state.

The subsequent 2026-09-01 line-vision image tightens adaptive black
classification from `Otsu + contrast/8` to `Otsu + contrast/16`; the 25/225
host fixture therefore lowers its threshold from 50 to 37. Host regression and
the full ESP-IDF build passed. The verified ESP32-S3 rev0.2 at MAC
`9c:cc:01:fb:8f:9c` accepted the bootloader, partition table, and application
with valid Flash hashes. The eight-second no-command check remained `IDLE`,
`cmd=0,0,0`, camera fresh with zero decode errors, and ultrasonic `VALID`. The
application is 0x59c70 bytes with SHA-256
`D609BDF6BD47F41F07109EB125E09A69F40C94211D511373C9B7CDC1DF6261E3`.

The preceding T-finish image added an IDLE-only `p` ASCII camera-view diagnostic and
calibrates the photographed T finish to width/area 800/200 with two decoded
candidate frames. Finish classification itself is hard-disabled from autonomy
startup through left strafe, forward motion, right strafe, and stationary line
confirmation. T-like shapes remain ordinary connected lines and cannot emit
`CAM=1111` during those phases. Only after `bypass_completed` is set does the
camera gate open with its candidate count cleared; a new two-frame T sequence
then enters latched `FINISHED` with zero motor policy. Synthetic normal-line,
thin-T, one-row artifact, temporal-reset, obstacle, and full host regressions
pass, as does the full ESP-IDF build. The verified ESP32-S3 rev0.2 at MAC
`9c:cc:01:fb:8f:9c` accepted all three images with valid Flash hashes. The
eight-second no-command check remained `IDLE`, `cmd=0,0,0`, camera fresh with
zero decode errors and zero camera drops. The real T endpoint measured
width 800--1000 and component area 452--490 permille, but correctly remained
ordinary `CAM=0010` while the finish gate was closed. The `p` command also
returned the actual 80x60 camera view. The active application is 0x5a040 bytes with
SHA-256
`B3589C3380A6536A4B57116B8AF1438CD471F8738CF23DC74D67A414CF1006EC`.

A stricter-shadow image is active with the additional absolute grayscale
ceiling of 120. Host tests prove that a connected grey-130 stripe on a
white-225 floor is rejected while a black-90 stripe remains detected; the full
ESP-IDF build also passes. The image is 0x5a060 bytes with SHA-256
`F30F62CA08FA579A5734DAE3BB74CEE0BC16C890A74DD9979A8F7F129EDCA401`.
After intermittent CP210x TX/sync failures, holding BOOT through reset allowed
the verified S3 to be identified, and a direct 115200-baud ROM no-stub write
completed with all three Flash hashes valid. The no-command check remained
`IDLE`, `cmd=0,0,0`, camera fresh with zero camera drops/errors, and reported
the expected capped `thr=120`. The previous large grey region disappeared;
only a 34--48 permille deep-black component remained. The 80x60 `p` view also
showed that the physical black tape had shifted to the central ROI's right
edge/outside it during wiring, so a centered-track placement is still required
before judging position calibration or autonomous steering.

The preceding timed-finish image kept the camera T-shape classifier compiled for
diagnostics and host coverage but leaves its runtime gate disabled. After the
right strafe reacquires black and completes five stationary confirmation cycles,
the supervisor enters `POST_FORWARD_500MS`, resumes normal camera line steering
for exactly 500 ms, and then latches `FINISHED` with zero motor policy. A `1111`
camera pattern during this interval is ordinary line input and cannot finish the
run early. Host tests cover the exact 500 ms boundary and ignored `1111`; the
full ESP-IDF build passes. The verified ESP32-S3 rev0.2 at MAC
`9c:cc:01:fb:8f:9c` accepted bootloader, partition table, and application by
115200-baud ROM no-stub flashing with all hashes valid. The eight-second
no-command check remained `IDLE`, `cmd=0,0,0`, with fresh camera data, zero
camera errors, and live ultrasonic readings. The application is 0x5a090 bytes
with SHA-256
`2CDABF3A141175416B6741C59A5B07A12F087399E32623D6CA3EA8021B9FB357`.

The preceding closer-obstacle / longer-finish build uses raw Echo 20--60 mm to
start avoidance, while 61 mm and above remains normal line following. The
1191 ms fixed forward segment is deliberately retained rather than reverting to
940 ms, giving about 40 mm more clearance beyond the obstacle. After right
strafe and five stationary line-confirmation cycles, the camera-steered final
forward interval is now 1000 ms before latched `FINISHED`; its telemetry state
name is `POST_FORWARD_1S`. Boundary host tests and the complete ESP-IDF build
pass. The 0x5a090-byte image has SHA-256
`551F581933F3C33E5E65F0D00FBBDD6E082EBD20710A532A86A0B189A82DC26C`.
After several intermittent UART failures, the verified ESP32-S3 rev0.2 at MAC
`9c:cc:01:fb:8f:9c` accepted bootloader, partition table, and application by
115200-baud ROM no-stub flashing; all three written-image hashes passed. The
eight-second post-reset check remained `IDLE`, `cmd=0,0,0`, with fresh camera
frames, zero camera drops/errors, and live ultrasonic readings around
298--340 mm.

The preceding 2026-09-02 build reduced the timed left segment from 1170 to
936 ms (20%), reduces the timed forward segment from 1191 to 715 ms (40%,
714.6 ms rounded to the nearest millisecond), leaves the right maximum at
960 ms, and reduces the post-reacquisition camera-steered interval from 1000
to 500 ms. Telemetry now calls the timed forward state `FORWARD_TIMED` and the
last interval `POST_FORWARD_500MS`. Host regression and the full ESP-IDF 5.4.4
build pass. The 0x5a090-byte application has SHA-256
`8B41BD2228E6AEB407169476AD3F0316BA465E9A6096DFDBD5D9FC6472EBB400`.
After the board was placed in download mode, COM3 was positively identified as
ESP32-S3 rev0.2 with 16 MB PSRAM and MAC `9c:cc:01:fb:8f:9c`. A 115200-baud ROM
no-stub write completed for bootloader, partition table, and application, and
all three write hashes verified. The ten-second no-command check remained
`IDLE`, `cmd=0,0,0`, with camera `fresh=1`, zero camera drops/errors, and valid
ultrasonic readings around 269--278 mm. A/B encoder counts still advanced
rapidly while motor commands remained zero, reconfirming electrical input noise;
do not use those counts for closed-loop distance control.

The subsequent current 2026-09-02 build keeps left strafe at 936 ms, increases
forward from 715 to 751 ms (5%, 750.75 rounded to the nearest millisecond),
increases the right maximum from 960 to 1008 ms (5%), and increases the final
camera-steered interval from 500 to 525 ms (5%). Telemetry names the last state
`POST_FORWARD_525MS`. Host regression and the full ESP-IDF build pass. The
0x5a090-byte application has SHA-256
`F254BC735F073088EDBC28D790F084DFE1C78AB6018C376603C5C5D9F6934CE6`.
The same verified ESP32-S3 rev0.2 at MAC `9c:cc:01:fb:8f:9c` accepted all three
images by 115200-baud ROM no-stub flashing, and every write hash verified. The
ten-second no-command check stayed `IDLE`, `cmd=0,0,0`, with camera `fresh=1`,
zero decode errors, and valid ultrasonic readings near 209 mm. Camera queue-drop
counts increased in the complex live scene but never made the snapshot stale.
B/C encoder counts again advanced rapidly at zero motor command, confirming the
known electrical noise remains.

The preceding 2026-09-02 right-slip build raises the obstacle trigger from
60 to 80 mm. It replaces the right strafe's 500-command, 150 ms launch boost
with a linear body-command ramp from 300 to 380 over 400 ms; Kiwi output rises
from approximately `+300/+460/-300` to `+300/+570/-300`. Left remains 936 ms,
forward remains 751 ms, and the post-reacquisition camera-steered interval
remains 525 ms. The right-search maximum is 1097 ms, selected to approximately
preserve the preceding 1008 ms profile's command integral despite the gentler
launch, and telemetry names the state `RIGHT_RAMP`. Host regression and the
full ESP-IDF 5.4.4 build pass. The 0x5a120-byte application has SHA-256
`95796DD905D5543A4B9A7073756FECA85D2C4CECCFD2C524EC2078E98D6657BF`.
COM3 was re-identified as the ESP32-S3 rev0.2 with 16 MB PSRAM and MAC
`9c:cc:01:fb:8f:9c`; the bootloader, partition table, and application were
written by 115200-baud ROM no-stub flashing and all three hashes verified. The
ten-second no-command check stayed `IDLE`, `cmd=0,0,0`, with fresh camera data,
zero camera drops/errors, valid ultrasonic readings around 215--219 mm, and
encoder counts `0,0,0`. No autonomous or motion command was sent.

The preceding fixed-finish build removes `LINE_CONFIRM` and all line-dependent
exits after obstacle detection. The first 20--80 mm Echo still suspends line
following at `BRAKE`; thereafter the supervisor no longer accepts line input.
It always completes left 936 ms, forward 751 ms, and the full ramped right
1097 ms, regardless of black, white, or `1111` camera patterns. It then applies
a fixed straight 500/400 start/steady override for 525 ms and latches
`FINISHED`; telemetry names this last motion `FINAL_FORWARD_525MS`. Camera
freshness remains a safety gate only before `BRAKE`, while near-obstacle and
repeated ultrasonic-uncertainty fail-safe stops remain active throughout the
fixed maneuver. Host regression and the full ESP-IDF 5.4.4 build pass. The
0x5a000-byte image has SHA-256
`010457CE7323F4DB99AB1D797BCEA8C9F281CCACB630FE514EE5988DCEA32BA2`.
An initial COM3 identity query returned an inconsistent ESP32-S2 result, so no
write was attempted; after automatic reset recovered, COM3 positively reported
ESP32-S3 rev0.2, 16 MB PSRAM, and MAC `9c:cc:01:fb:8f:9c`. Bootloader,
partition table, and application were then written by 115200-baud ROM no-stub
flashing and all three hashes verified. The ten-second no-command check stayed
`IDLE`, `cmd=0,0,0`, with fresh camera data, zero decode errors, and valid
ultrasonic readings around 501--527 mm. In the complex scene, camera queue drops
rose from 1 to 4 and control overruns from 2 to 9 without making the frame stale.
A/C encoder counts again advanced rapidly at zero command; no motion command was
sent and the encoder noise must not be used for closed-loop distance control.

The preceding left-distance build increased the fixed left strafe by 10% from
936 to 1030 ms (`936 * 1.10 = 1029.6`, rounded to the nearest millisecond).
Forward remained 751 ms, the full ramped right remained 1097 ms, final fixed
straight motion remains 525 ms, and no line-dependent path is restored after
`BRAKE`. Host regression and the full ESP-IDF 5.4.4 build pass. The 0x5a000-byte
application has SHA-256
`CF7B01829EFC6F23AEB17BC4A743DF978AEAB3A56283D052E37E9A6001C5BE4B`.
The original CP210x USB location `1-4` disconnected while the bootloader was
about 23% written, leaving a partial startup image; slower retries on that link
also failed. After moving the USB connection to location `1-1`, COM3 positively
reported ESP32-S3 rev0.2, 16 MB PSRAM, and MAC `9c:cc:01:fb:8f:9c`. A complete
57600-baud ROM no-stub recovery write replaced bootloader, partition table, and
application, and all three hashes verified. The ten-second no-command check
stayed `IDLE`, `cmd=0,0,0`, with camera fresh and zero camera drops/errors;
ultrasonic readings were mainly about 193 mm. B/C encoder counts advanced at
zero command, while control overrun and diagnostic-drop counts remained 1. No
motion command was sent.

The preceding forward-distance build increased the fixed middle forward segment
by 10% from 751 to 826 ms (`751 * 1.10 = 826.1`, rounded to the nearest
millisecond). Left remains 1030 ms, the full ramped right remains 1097 ms, and
the final fixed straight segment remains 525 ms. Host regression and the full
ESP-IDF 5.4.4 build pass. The 0x5a000-byte application has SHA-256
`B326B4C6D0F16D141A03E18EB5E6016349A17FA2FC2A56D06A86C799DF92A1B8`.
After repeated CP210x disconnects and recovery from a partially erased
bootloader, COM3 at USB location `1-6` was driver-restarted and positively
identified the target as ESP32-S3 rev0.2 with 16 MB PSRAM and MAC
`9c:cc:01:fb:8f:9c`. A complete 115200-baud ROM no-stub write replaced the
bootloader, partition table, and application; all three Flash hashes verified.
The ten-second no-command check stayed `IDLE` with `cmd=0,0,0`, fresh camera
frames, zero camera decode errors, and ultrasonic readings around 234 mm.
Camera queue drops rose from 2 to 7 and control overruns from 5 to 8 without
making frames stale. C encoder counts advanced rapidly at zero command, matching
the known electrical-noise issue; no motion command was sent.

The preceding right-launch anti-yaw build kept the 300-to-380 lateral ramp,
400 ms ramp time, and 1097 ms right-segment duration. It adds a signed
counter-clockwise body command that fades from -60 to zero across the same
400 ms. Because lateral A/C dead-zone floors must remain active when this small
yaw term is present, the floor condition now covers any zero-forward lateral
motion. The resulting launch/midpoint/steady wheel vectors are approximately
`+300/+510/-300`, `+300/+540/-300`, and `+300/+570/-300`; this specifically
targets the observed initial clockwise body rotation without changing the
steady right output or any later avoidance state. Host regression and the full
ESP-IDF 5.4.4 build pass. The 0x5a040-byte application has SHA-256
`4AC81E441051D754AAE79ADD74ACA6530B42350347ACD23387A946CC4F4F6B05`.
COM3 at USB location `1-6` identified ESP32-S3 rev0.2, 16 MB PSRAM, and MAC
`9c:cc:01:fb:8f:9c`; after restarting the intermittent CP210x bridge, a complete
115200-baud ROM no-stub write verified all three Flash hashes. The ten-second
no-command check remained `IDLE`, `cmd=0,0,0`, with fresh camera frames, zero
decode errors, and ultrasonic readings around 549--557 mm. Camera drops rose
from 3 to 4, control overruns from 2 to 5, and C encoder noise from about -12 to
-34 counts; no motion command was sent.

The current forward-distance retune increases the fixed middle forward segment
by 30% from 826 to 1074 ms (`826 * 1.30 = 1073.8`, rounded to the nearest
millisecond). Left remains 1030 ms, the anti-yaw right segment remains 1097 ms,
and the final fixed straight segment remains 525 ms. Host regression and the
full ESP-IDF 5.4.4 build pass. The 0x5a040-byte application has SHA-256
`EC33ACE96251D24132A43D9829447BBE5392C7CAAB45FCA9A3A04FAB46BCA3F6`.
COM3 at USB location `1-6` positively identified ESP32-S3 rev0.2, 16 MB PSRAM,
and MAC `9c:cc:01:fb:8f:9c`; after restarting the intermittent CP210x bridge, a
complete 115200-baud ROM no-stub write verified all three Flash hashes. The
ten-second no-command check remained `IDLE`, `cmd=0,0,0`, with fresh camera
frames and zero decode errors. Ultrasonic samples remained valid while the
observed scene changed between roughly 160, 380, and 553 mm. Camera drops rose
from 3 to 8 and control overruns from 2 to 6 without making frames stale;
diagnostic drops remained 1 and all encoder counts remained zero. No motion
command was sent.

For live-view-only testing, the board temporarily ran the standalone
`experiments/camera_wifi_preview` image instead of the main application. It
forces every motor enable, PWM, and direction output low at entry and contains
no active line-follow or obstacle-bypass path. Its application is 0xd2f70 bytes
with SHA-256
`1B46F7F465AF93FF4185C64639A05CAA78915091517AD5AA7C84D172856B7654`.
The verified ESP32-S3 accepted the bootloader, partition table, and application
at 115200 baud in ROM no-stub mode, with all three write hashes valid. Startup
reported SoftAP `SmartCar-Camera` at `192.168.4.1`, negotiated MJPEG 640x480 at
15 fps, and emitted `CAMERA_PREVIEW_RESULT=READY`; periodic counters confirmed
continuous frame capture with no viewers connected.

The board now runs the restored 0x5a040-byte `build/smart-car-test.bin` main
application again. COM3 re-confirmed ESP32-S3 rev0.2, 16 MB PSRAM, and MAC
`9c:cc:01:fb:8f:9c`; a 115200-baud ROM no-stub write verified the bootloader,
partition table, and application hashes. A 12-second no-command check remained
`IDLE`, `obstacle=0`, `cmd=0,0,0`, and encoders `0,0,0`, with fresh camera data,
zero decode errors, and valid ultrasonic updates. No start or motion command
was sent.

During the next controlled line-follow run, a rear-right hairpin produced line
loss and the left/right alternating search. Physical observation showed that
the left search translated the chassis off track. Source review confirmed the
cause: the old `A=-211,B=0,C=+211` search vector is a yaw-plus-lateral-motion
combination on a three-wheel Kiwi chassis, not pure yaw. The search now uses the
previously field-verified pure-yaw basis at 420: left is
`A=-420,B=+420,C=+420`, and right is its exact inverse. It bypasses the normal
500/150 ms A/C drive assist. To approximately preserve the old angular sweep,
legs are now 380, 700, 1020, 1340, 1660, and 1980 ms, then capped at 2000 ms.
Host regression and the full ESP-IDF 5.4.4 build pass. The new application is
0x5a010 bytes with SHA-256
`F5F12E09FB94D2E834371428E105E9339E3910BB5266CD794B6EBDBB6CE93C31`.
COM3 re-confirmed the target as ESP32-S3 rev0.2 with 16 MB PSRAM and MAC
`9c:cc:01:fb:8f:9c`; 115200-baud ROM no-stub flashing verified the bootloader,
partition table, and application hashes. The board now runs this image. A
12-second no-command check remained `IDLE`, `obstacle=0`, `cmd=0,0,0`, and
encoders `0,0,0`, with fresh camera data, zero decode errors, and valid
ultrasonic readings near 773 mm. No start or motion command was sent.

The current direct-turn-test build adds `y` as the exact bounded counterpart
to the existing `t` command: `y` applies the left pure-yaw vector
`-420/+420/+420`, while `t` applies `+420/-420/-420`; each runs for 100 ms and
then automatically returns to `IDLE` with zero motor output. Host regression
and the full ESP-IDF 5.4.4 build pass. The application is 0x5a060 bytes with
SHA-256
`FE630FAF5D6405D24961C7632C656F13504E25186947D4A7C57D05C6EEEA7B4A`.
COM3 re-confirmed ESP32-S3 rev0.2, 16 MB PSRAM, and MAC
`9c:cc:01:fb:8f:9c`; 115200-baud ROM no-stub flashing verified all three image
hashes. An eight-second no-command check stayed `IDLE`, `obstacle=0`,
`cmd=0,0,0`, and encoders `0,0,0`, with fresh camera frames, zero decode
errors, and live ultrasonic data. With all wheels raised, `y` produced final
encoder counts `-249/-190/-193`, and `t` produced `+240/+181/+213`. Both tests
completed automatically, returned to `IDLE` and `cmd=0,0,0`, and a final `x`
was also sent.

The last fully installed application adds an integrated CP210x camera monitor
without changing the motion-control behavior above. Command `u` acknowledges at
115200 baud and switches UART0 to 460800 baud for CRC32-protected 80x60 RGB332
preview packets at about 5 fps; `z`, or a three-second heartbeat timeout,
restores 115200 baud. `tools/usb_camera_monitor.py` displays the image, highlights
the exact adaptive-threshold black pixels in red, draws the ROI in yellow and
near/far centers in green/blue, and reports normal controller telemetry. It never
sends `f`, sends `z` when closed, and exposes an `x` emergency-stop button. Host
tests, its parser self-test, Python compilation, and the full ESP-IDF 5.4.4 build
pass. The installed application is 0x5aad0 bytes with SHA-256
`1BDB98F5A0164B86DBB5E7149441E2031D21FCA2A0FFC9FA1886B32F204F2A96`;
all three ROM no-stub flash hashes verified on COM3. A live five-second probe
received 24 frames and nine status records (sequence 1410 through 1479) with
zero CRC errors. Before and after preview, the board remained `IDLE` with
`cmd=0,0,0` and encoders `0,0,0`, and a final `x` was sent.

The subsequent current source adds the path-ordered component tracer described
above. Mirrored synthetic rear-right and rear-left hairpins prove that the entry
remains near the vehicle center while the far point follows the returning leg;
all prior line, obstacle, kinematics, preview-packet, and hardware-policy host
regressions also pass. The complete ESP-IDF 5.4.4 build is 0x5afc0 bytes with
SHA-256
`9EE9FCD7F70B51CD2BE3C3D98FFBB9AA08E47389F8F108406F4E2CE0D940B024`.
This image is not installed: its bootloader and partition table verified, and
application sectors 1--25 of 91 verified independently at 4 KiB each, but the
CP210x link failed during sector 26. The user explicitly stopped flashing; all
flash processes were terminated and COM3 was released. Treat the board's
application region as incomplete and do not run motion tests until the complete
application is written, whole-image-verified, hard-reset, and checked at
`IDLE`, `cmd=0,0,0`.

The current development environment uses ESP-IDF at
`C:\esp\v5.4.4\esp-idf` and Espressif tools under `C:\Espressif\tools`.
These are machine-local paths and are not committed as VS Code settings.

The superseded 2026-09-03 fail-stop recovery build reduced autonomous line-search yaw
from 420 to 360, fixes every recovery's first leg to left before alternating
right/left, and treats the first nonzero direction opposite an established
lock as an immediate zero-output line loss. Reacquisition still needs three
distinct decoded frames, and they must now share the same left/center/right
direction; alternating candidates reset the count to one. Host regression and
the full ESP-IDF 5.4.4 build pass. The application is 0x5b030 bytes with
SHA-256 `C54D7371A18CA5233F9DC4B613AD51CC729D1C9B58DDC697488AD46A7B307462`.
Several writes failed on the intermittent CP210x link and left a partial
application, but after reconnecting COM3 at USB location `1-1`, a complete
115200-baud ROM no-stub application write reached 100% and its whole-image
hash verified. The retained verified bootloader and partition table then booted
the new image. An eight-second no-command check stayed `IDLE`, `obstacle=0`,
and `cmd=0,0,0`, with fresh camera frames, zero camera drop/decode errors, and
valid ultrasonic readings near 110 mm. C encoder counts advanced at zero motor
command, matching the known electrical-noise issue; no motion command was sent.

Later on 2026-09-03, the user requested that the complete line-following
behavior return to the last 2026-09-01 local Git commit, `9354358` at 17:30:03.
`line_follow.c`, `line_follow.h`, and `camera_line_vision.c` now match that Git
tree byte-for-byte. The line configuration is also restored to search command
211 and 1200/2400/7200 ms expansion. The newer obstacle supervisor, red-ball
observer, direct-turn self-test, and USB preview remain intact. Two line-result
Y fields are retained only as a preview ABI extension and do not affect any
decision. Host regression and the full ESP-IDF 5.4.4 build pass. The resulting
application is 0x5ab50 bytes with SHA-256
`F5EF40BC8C8C9C500587BAF769BB64234019D1D9964E8C5AEDA851A92DD2103B`.
COM3 positively identified the target as ESP32-S3 rev0.2 with 16 MB PSRAM and
MAC `9c:cc:01:fb:8f:9c`. A complete 115200-baud ROM no-stub write installed the
bootloader, partition table, and application; all three device-side hashes
verified and the board hard-reset. An eight-second passive check sent no
commands and stayed `IDLE`, `obstacle=0`, and `cmd=0,0,0`; camera frames remained
fresh with increasing sequence numbers, zero drops and zero decode errors, and
ultrasonic readings near 382--391 mm. C encoder counts rose from about 118 to
167 at zero motor command, matching the known electrical-noise issue. This 9.1
line-logic image is now installed.

The subsequent source adds a one-shot fixed entry maneuver before line
following on every `BOOT`/`f` start. After ultrasonic clear authorization it
drives forward for 1074 ms at 400 (500 for the first 150 ms), stops for 150 ms,
applies the verified `+420/-420/-420` pure clockwise wheel vector for 400 ms,
stops for 150 ms, resets line history from the latest camera frame, and then
enters normal 9.1 line following. The 20 cm and 120 degree labels are open-loop
estimates based on existing field timings; motor-on encoders remain unsuitable
for closing the distance or angle. The obstacle supervisor and all top-level
stop/fault/manual arbitration remain able to preempt the sequence. UART status
adds `startup=0..5`, and the PC camera monitor decodes those phase names. Host
regression and the complete ESP-IDF 5.4.4 build pass. The 0x5b030-byte
application has SHA-256
`B66139BC109AD6CF3B74C65595AB023CB93BA9E705301C9F85F1A74DB00A37A3`.
COM3 re-identified the board as ESP32-S3 rev0.2 with 16 MB PSRAM and MAC
`9c:cc:01:fb:8f:9c`. A complete 115200-baud ROM no-stub write installed the
bootloader, partition table, and application; all three hashes verified and
the board hard-reset. An eight-second passive check sent no commands and stayed
`IDLE`, `startup=0`, `obstacle=0`, and `cmd=0,0,0`. Camera frames stayed fresh
with zero decode errors; drops rose from 4 to 8 without stale data. Ultrasonic
readings recovered from one initial no-return to valid samples around 249--624
mm. Encoder A drifted only about -5 counts at zero command. This fixed-entry
image is now installed, but its motion sequence has not been triggered or
physically distance/angle-verified.

The subsequent source reduces only the fixed-entry clockwise turn from 400 ms
to 267 ms, exactly two thirds after integer rounding. Speed remains 420 and the
wheel vector remains `+420/-420/-420`, so the open-loop angle estimate changes
from about 120 degrees to about 80 degrees; the forward and settling phases are
unchanged. Host regression, monitor parser self-test, and the complete ESP-IDF
5.4.4 build pass. The resulting 0x5b030-byte application has SHA-256
`C1C7CD0EE58B7224D048BB9667EBB68F5CD8F4DDAD2AB2869C9B641A51991F6C`.
COM3 re-identified the board as ESP32-S3 rev0.2 with 16 MB PSRAM and MAC
`9c:cc:01:fb:8f:9c`. A complete 115200-baud ROM no-stub write installed the
bootloader, partition table, and reduced-turn application; all three write-side
hashes verified and the board hard-reset. An approximately eight-second passive
check sent no commands and stayed `IDLE`, `startup=0`, `obstacle=0`, and
`cmd=0,0,0`; camera sequences increased with zero decode errors. The 267 ms
image is now installed, but its fixed-entry motion has not been triggered or
physically angle-verified.

The latest source reduces only that 267 ms fixed-entry turn to three quarters:
200.25 ms rounded to 200 ms. Speed remains 420 and the wheel vector remains
`+420/-420/-420`, so the time-based angle estimate is now about 60 degrees.
Forward and settling phases are unchanged. Host regression, monitor parser
self-test, and the complete ESP-IDF 5.4.4 build pass. The resulting 0x5b030-byte
application has SHA-256
`CE5FD958BA40A89031A5F86F3D1A36BFA6B9640551FCFA3F5880CA8358EE8F2A`.
COM3 re-identified the board as ESP32-S3 rev0.2 with 16 MB PSRAM and MAC
`9c:cc:01:fb:8f:9c`. A complete 115200-baud ROM no-stub write installed all
three images, verified each write-side hash, and hard-reset the board. An
approximately eight-second passive check sent no commands and stayed `IDLE`,
`startup=0`, `obstacle=0`, and `cmd=0,0,0`, with all three encoders at zero;
camera sequences increased with zero decode errors. The 200 ms image is now
installed, but its fixed-entry motion has not been triggered or physically
angle-verified.

The latest source redesigns the read-only red-ball detector for the live
center-frame ball with specular glare. It uses strong-red-seeded weak-red
hysteresis, two bounded local highlight-repair passes, strong-seed ratio and
mean-color safeguards, and confidence-based component selection. The preview
now marks candidates in orange and confirmed balls in magenta over the entire
frame, independently of the yellow line ROI, and telemetry adds strong/highlight
counts plus the exact box. Synthetic tests cover a weak-edged ball with a white
highlight above the line ROI, a neutral highlight without a red seed, weak red
background, edge clipping, and elongated objects. Host regression, monitor
parser self-test, and the complete ESP-IDF 5.4.4 build pass. The resulting
0x5b710-byte application has SHA-256
`809274167B3ADAC82649B98583F341F8B6E96B42EA4BED25343D7C2C6F7E28C8`.
This image was flashed to the ESP32-S3 on COM3 at 460800 bit/s on 2026-09-03;
the bootloader, partition table, and application all passed esptool write-back
hash verification before a hard reset. A subsequent 115200-bit/s UART check
showed advancing camera sequences, `cam_err=0`, and the new `bseed`/`bbox`
telemetry, confirming that the strong/weak-red image is running on the board.
The GUI monitor was stopped for flashing and was not reopened automatically.

The newest source adds a standalone red-ball align-and-capture mode without
chaining it to obstacle `FINISHED`. Command `b` is accepted only from `IDLE`;
it searches left for 800 ms, stops, searches right for 1600 ms, stops on a
candidate, aligns with 80 ms A/C turn pulses separated by 120 ms observations,
then approaches at 300/250/210 after a 360-command, 100 ms launch boost. A
30%-reduced 210/175/147/252 experiment was reverted because it could not
reliably overcome static friction; search and alignment speeds are unchanged.
The calibrated clip axis is x=+25. The first
frame satisfying y>=790, bbox bottom>=52, and height>=170 or area>=18 stops
immediately; a second stationary frame latches `BALL_DONE`. Stale vision,
12-second approach timeout, or 30-second total timeout latches failsafe. A
40 cm component may use a separate 2-permille/900-confidence gate with five
frames, while normal candidates retain the 4-permille/three-frame rule. Host
tests cover search order, direction reversal braking, P10 non-trigger, P0 stop
and stationary confirmation, and stale-camera failsafe. Monitor syntax/parser
tests and the complete ESP-IDF 5.4.4 build pass. The resulting 0x5c700-byte
application has SHA-256
`E54CAF7DDF5EC6CD1D6F250AC25FDA90DFD6C41F364E78F0663076F28DA7B990`.
This image was flashed to the ESP32-S3 on COM3 at 460800 bit/s on 2026-09-03;
the bootloader, partition table, and application all passed post-write hash
verification and the board hard-reset successfully. Read-only startup telemetry
confirmed `mode=IDLE`, `cmd=0,0,0`, `cam_err=0`, and the new `ballctl`/`bfar`
fields. No `b` motion command has been sent and ground-motion testing remains
pending. The preceding 0x5b710 image is no longer installed.

The installed full-resolution build switches the JPEG decoder from 1/8 scale to native
640x480 RGB888. Red-ball segmentation and connected components consume all
307200 pixels; line vision consumes the native lower 640x240 half before the
existing x=25%..75%, y=60%..93% ROI is applied. Both connected-component queues
now use 32-bit indices. Ball boxes and the capture-bottom gate use normalized
permille coordinates; the former 52-of-60 capture gate is represented as 880
permille. The monitor wire format stays at 80x60 RGB332 and is produced by block
downsampling the native frame, so its baud rate and packet ABI do not change.
The full-resolution decode task runs below the motion controller priority so a
slow vision pass cannot starve stop/fault arbitration. Original host regression,
a native-VGA ball case, scaled-preview testing, monitor parser testing, and the
complete ESP-IDF 5.4.4 build pass. The final 1500-ms-freshness build is a
0x5ca00-byte application with SHA-256
`686A0C8C615E984F528DF4ACFDC58F21F838E55EA6EEA5875A583ED6C870D366`.
It was flashed to the ESP32-S3 on COM3 at 460800 bit/s on 2026-09-03. The
bootloader, partition table, and application all passed post-write hash
verification and the board hard-reset successfully. A 15-second passive check
received 30 status records with no fault or watchdog text; every record stayed
`mode=IDLE`, `fresh=1`, `cam_err=0`, and `cmd=0,0,0`, while decoded sequence
advanced from 5 to 20. The live scene's red ball remained confirmed. No motion
command was sent, and the updated GUI monitor was reopened after verification.
The old 80x60 calibration must still be rechecked at native resolution.

The first native-resolution flash booted and allocated its PSRAM workspaces,
remained `IDLE` with `cmd=0,0,0`, and produced valid zero-error analyses, but
the initially scaled 16-pass highlight repair took roughly four seconds per
frame, accumulated hundreds of UVC drops, and triggered the task watchdog.
That runtime trial is superseded by the bounded two-pass source above and must
not be treated as an acceptable motion build.

The bounded two-pass native build removed the watchdog condition and confirmed
continuous full-resolution analysis with `cam_err=0`, but measured only about
one analyzed frame per second. The camera freshness interval is therefore 1500
ms in the final full-resolution source instead of the old 350 ms; this keeps
the deliberately slow accuracy-first mode usable while retaining a bounded
camera-loss stop. Ground motion must remain low-speed until this latency is
physically evaluated.

The preceding installed source selected the intermediate 320x240 RGB888 decode
profile (`JPEG_IMAGE_SCALE_1_2`). Ball vision consumes the complete 320x240
frame and line vision consumes its lower 320x120 half before applying the same
normalized ROI. Ball and line workspaces shrink with those dimensions while
the normalized box/capture ABI and 80x60 UART preview remain unchanged. The
freshness bound is reduced from 1500 to 500 ms. Host tests
cover the exact 320x240 ball and line shapes, all prior regressions pass, the
monitor syntax/parser tests pass, and the complete ESP-IDF 5.4.4 build passes.
The 0x5c9e0-byte application has SHA-256
`74BBCE00A673ABF7065FA3F1BCAB889B29CEA9FAEBBD45FB263BF8F873DD1019`.
It was flashed to the ESP32-S3 on COM3 and all three written regions passed
esptool hash verification. A 10-second live-preview probe received 26 frames
and 20 status records with zero CRC errors (about 2.6 processed FPS). A separate
passive status check remained IDLE with zero motor commands, `fresh=1`, and
`cam_err=0` throughout.

The preceding installed build retained that 320x240 pipeline but raised camera
freshness from 500 to 1000 ms. Live processing measured about 2.2--2.6 FPS, so
the former 500 ms bound could catch ordinary decode/scheduling jitter only
after BOOT enabled the autonomous camera-freshness guard and latch a camera
timeout for the rest of that boot. Host regression, monitor parser, and the
complete ESP-IDF 5.4.4 build pass. The 0x5c9e0-byte application has SHA-256
`F60D591A17BBCB1CB5952890D755A2661209E3416D92E6B0DB7D221C47D3D557`.
It was flashed to COM3 with all three regions hash-verified. A 12-second passive
check produced 24 IDLE status records, decoded sequence 34 through 59, no fault,
`fresh=1`, `cam_err=0`, and zero motor commands throughout. Autonomous motion
was not started during this verification.

The preceding installed source was fully restored to the prior 80x60 vision
pipeline without removing the later startup maneuver, 9.1 line control,
obstacle timing, robust red/blue-target detector, or standalone ball approach. It uses
`JPEG_IMAGE_SCALE_1_8`, feeds the complete 80x60 frame to ball vision and the
lower 80x30 half to line vision, restores 16-bit component queues, and restores
the 350 ms freshness bound. Host regression and the complete ESP-IDF 5.4.4
build pass. The current 0x5cc20-byte application has SHA-256
`73A5EF6D6EA1FE97FB4D8AB3DA80566EC7D01896AEB448D89FAB211C198A9472`.
Its standalone ball-approach forward levels are restored to 300/250/210 with a
360-command, 100 ms launch boost because the 30%-reduced experiment could not
reliably overcome static friction.
It was flashed to COM3 at 460800 bit/s; bootloader, partition table, and
application all passed write-time hash verification before the automatic hard
reset. An eight-second passive check produced 16 status records and remained
IDLE with `fresh=1`, `cam_err=0`, and zero motor commands. Decoded sequence
advanced from 53 to 161. The centered blue rectangle was continuously reported
as `BALL=3/1/1`, with center around x=-51/y=493 permille and confidence about
993 permille. No motion command was sent during verification.

The current installed build retains the 80x60 pipeline and replaces the former
red/blue best-candidate output with simultaneous independently confirmed red
ball and blue goal observations. The standalone `b` mode now performs bounded
route pre-alignment, red-ball approach/contact confirmation, blue-goal push
alignment, visually steered pushing, and two-frame stationary goal-overlap
confirmation. Red or blue loss stops motion; red loss after approach starts is
latched failsafe, and route, approach, push, and total timers are bounded. Host
regression, monitor syntax/parser tests, and the ESP-IDF 5.4.4 build pass. The
0x5dfc0-byte application SHA-256 is
`68555F37623C3C96697F3240F3C12FE3736D87D007D1FAF464FAA2FEC51AB332`.
COM3 was re-identified as ESP32-S3 rev0.2 with 16 MB PSRAM and MAC
`9c:cc:01:fb:8f:9c`; bootloader, partition table, and application passed a
separate no-stub readback digest verification after flashing. An eight-second
preview probe received 37 frames and 16 status messages with zero CRC errors.
A following six-second passive check remained `IDLE`, `fresh=1`, `cam_err=0`,
and `cmd=0,0,0`; decoded sequence advanced 616 to 691. The centered blue goal
was independently stable as `GOAL=3/1/1/255` around x=20..25/y=493 permille
with confidence 993, while no red ball was accepted (`BALL=2/0/0/0`). No `b`
or other motion command was sent.

The current capture-tolerance build addresses the first live `b` test stopping
at `BALL_FAILSAFE` with `capture_frames=1`. That frame still had a confirmed
red ball at y=814/bottom=898, but height=150 and area=14 missed the former
170/18 secondary size gates. Capture now keeps the position gates y>=790 and
bottom>=880 while relaxing size to height>=130 or area>=10. It accumulates two
passing observations within five stationary frames; an intervening visible
non-passing frame stays stopped instead of failing immediately. Host regression
and the complete ESP-IDF build pass. The 0x5e040-byte application SHA-256 is
`0CD2A5EB8F4E716A817AD2CCFAA476BCFC27751C9DDD6FA2408A68E818DEFACF`.
It was flashed to the verified ESP32-S3 on COM3 and all three images passed
independent readback verification. A five-second passive check remained IDLE,
fresh, error-free, and at zero motor command; the live red observation at
y=824/bottom=898 had height=133 and now passes the relaxed size gate. The
updated debug window was reopened without sending `b`.

The current ball-recovery build responds to the user's request to avoid a
latched ball-mode failsafe.  Every former ball-policy anomaly path (stale
camera, temporary red/blue loss, approach/push timeout, and total timeout) now
stops the motors immediately, enters `BALL_RECOVERY_WAIT` for 2000 ms, and
restarts clean acquisition only when camera data is fresh.  Hardware-level
fault and emergency-stop handling remain intact.  `ballctl` now appends the
persistent last transition reason so future stops are diagnosable from normal
status telemetry.  A second live clip observation at y=780--791 and box bottom
864 also showed that the former 790/880 position gates were still too strict;
they are now 760/850, while P10 remains clearly separated at about y=651 and
bottom=729.  Host regression, monitor parser checks, and the full ESP-IDF 5.4.4
build pass.  The 0x5e120-byte application SHA-256 is
`4461A6C5089E2455500235BB40E49972DEF847A45993ED4046F67C8A8D2B5FF5`.
COM3 was re-identified as ESP32-S3 rev0.2 with 16 MB PSRAM and 32 MB octal
flash; bootloader, partition table, and application all passed independent
readback verification.  A six-second passive check received 12 IDLE status
records with fresh camera data, no camera error, and zero motor commands.  In
the following user-started live run, the relaxed contact gate reached cap=2
and `BALL_PUSH`; push timeout transitioned to recovery wait, paused for two
seconds, and restarted search without entering `BALL_FAILSAFE`.  The debug
window remains open.

The current push-breakaway build raises steady ball pushing from 210 to 250
and adds a 360-command, 100 ms boost each time motion enters `BALL_PUSH`,
including after a visual realignment.  Host regression explicitly checks the
boost and post-boost commands; monitor parsing and the full ESP-IDF 5.4.4 build
also pass.  The 0x5e1b0-byte application SHA-256 is
`E11283CCDC41244D14679F8D455BE760D78087DA037F6BA8B61D08862663280A`.
COM3 was re-identified as the same ESP32-S3 rev0.2 / 16 MB PSRAM / 32 MB octal
flash target, and all three flashed regions passed independent readback
verification.  A six-second passive check produced 12 IDLE status records,
all fresh and error-free with zero motor commands.  An authorized eight-second
`b` trial then reached route alignment and brief approach, but the blue goal
lost confirmation (`GOAL=3/0/0/0`) before capture, so it stopped, waited, and
retried rather than reaching `BALL_PUSH`.  The harness always sent `x` at the
end and confirmed `IDLE/cmd=0,0,0`.  A second authorized ten-second trial began
with both observations stable at 255, completed route/red alignment, confirmed
capture, and reached `BALL_PUSH` for about 2.2 seconds.  Live steady commands
ranged from about -253/0/-247 to -322/0/-178 as blue-goal steering changed;
red center-y advanced from roughly 700 to 833 and the encoder readings changed,
confirming that 360/250 overcame static friction and produced physical motion.
The time-bounded harness then sent `x` and confirmed `IDLE/cmd=0,0,0`, with no
camera/runtime errors.  The debug window was reopened.

The current deferred-goal-search build no longer blocks red-ball approach when
`left_target` is absent.  A confirmed red ball proceeds through yaw alignment,
decreasing-speed approach, and two-frame clip/contact confirmation without a
blue observation.  Only then does it alternate bounded left and right yaw
sweeps, with a stopped frame between directions, until `left_target` is found;
the independently displayed `right_target` never satisfies delivery-target
acquisition.  If a previously visible target disappears during route
pre-alignment, the controller falls through to red-ball approach instead of
waiting.  Target loss during push alignment or pushing stops the current
command first and enters the same post-capture target search.  Host regression
tests exercise the complete no-target approach/capture/search/reacquisition
path, and monitor parsing plus the full ESP-IDF 5.4.4 build pass.  The
0x5e9b0-byte application SHA-256 is
`CE99F9454ACD3983432C7A6A29118DDD1E92A6111FFF5A9932EFC3F0E2EA074F`.
COM3 was re-identified as the same ESP32-S3 rev0.2 / 16 MB PSRAM target; all
three flashed regions passed independent readback verification.  A ten-second
passive check stayed `IDLE`, fresh, camera-error-free, and at `cmd=0,0,0`.
That live scene intentionally had a confirmed red ball and only the right blue
target (`LEFT_TARGET=0/0/0/0`, `RIGHT_TARGET=3/1/1`), matching the newly handled
starting condition.  No `b` or other motion command was sent.

The current mid-approach target-reacquisition build keeps the deferred search
behavior above, but no longer insists on reaching the clip once `left_target`
becomes visible.  While red alignment or forward approach is active and route
pre-alignment has not completed, a newly confirmed left target immediately
zeros the current command and returns to `BALL_ROUTE_ALIGN`.  Discovery during
a yaw pulse passes through `BALL_ROUTE_SETTLE` first, preventing an abrupt
direction change.  If the target disappears again before route alignment
finishes, red-ball approach resumes and remains eligible for another later
reacquisition.  Host regression covers appearance during forward motion and
loss during route alignment; the complete ESP-IDF build passes.  The
0x5ea90-byte application SHA-256 is
`C793B5499BCD8F4874BC55BE448F61C29C9DE1C55181FF46F68AC2272859FBE9`.
The verified ESP32-S3 on COM3 accepted the image and all three regions passed
independent readback verification.  A nine-second passive check remained
`IDLE`, fresh, camera-error-free, and at `cmd=0,0,0`, with red ball plus both
blue targets confirmed.  No motion command was sent.

The current pulsed red-ball search build replaces continuous lost-ball yaw
with a repeated short-step pattern: turn quickly at command 360 for 80 ms,
stop for a full 150 ms camera observation, then continue in the same direction.
Accumulated motor-on time preserves the previous search coverage: 800 ms left
before changing direction and 1600 ms right before changing back.  The
post-capture blue-goal search remains the gentler continuous command 260 and is
not affected by this tuning.  Host regression explicitly checks pulse timing,
the stationary observation interval, accumulated sweep timing, and direction
reversal; the full ESP-IDF 5.4.4 build passes.  The 0x5eb00-byte application
SHA-256 is
`08531C533195672673B8E1724D16005BE413D2659A34979D7D3A0DA65C498BB5`.
COM3 was re-identified as ESP32-S3 rev0.2 with 16 MB PSRAM and 32 MB flash;
bootloader, partition table, and application all passed independent readback
verification.  A nine-second passive check stayed `IDLE` with fresh camera
data, `cam_err=0`, and `cmd=0,0,0`.  No motion command was sent.

The current pulsed-yaw / stronger-push build applies the short-step pattern to
every pure yaw operation in standalone ball mode: red-ball search, red-ball
alignment, post-capture left-target search, and push-heading alignment.  Each
step turns for 80 ms and then holds zero motor output for 200 ms so fresh camera
observations are collected while stationary.  Search still accumulates 800 ms
of left motor-on time and 1600 ms right before reversing; left-target search
retains its gentler 260 command while red search remains 360 and alignment 300.
The lateral route-shift is not a yaw operation, but its existing 90 ms pulse now
also receives the longer shared 200 ms observation interval.  Loaded pushing
now starts with command 420 for 180 ms and continues at 300, replacing the
former 360-for-100-ms / steady-250 profile.  Host regression covers both
pulsed search paths, alignment timing, and both push-speed phases; the full
ESP-IDF 5.4.4 build passes.  The 0x5eb30-byte application SHA-256 is
`16409A90623D049EFFABC0950CD0D40C843FE396123ED804C84D7A61F4FDDC64`.
COM3 was re-identified as the ESP32-S3 rev0.2 / 16 MB PSRAM target, and
bootloader, partition table, and application all passed independent readback
verification.  A nine-second passive check remained `IDLE`, fresh,
camera-error-free, and at `cmd=0,0,0`; no movement command was sent.

The current strict-overlap / lateral-goal-alignment build removes the former
60-permille expansion from the delivery test: the detected red-ball center must
be inside the detected left-target box itself on three consecutive stationary
frames before `BALL_DONE`.  Pre-capture ball/target route alignment remains a
kiwi lateral translation, and post-capture `BALL_PUSH_ALIGN_PULSE` now also
uses lateral translation rather than pivoting the ball.  Both directions use
repeated 80 ms command-300 lateral steps followed by 200 ms at zero output;
red-ball centering and visual search remain pulsed yaw operations.  Host
regression proves that a red center 8 permille outside the blue box no longer
finishes, checks all three overlap confirmations, and verifies the rightward
lateral wheel vector and pulse/settle timing.  The full ESP-IDF 5.4.4 build
passes.  The 0x5eb10-byte application SHA-256 is
`DB3E00C97A3386357D28F02C969F24BE1321D5F5D59C46756E389ABF90D2C61D`.
COM3 was re-identified as the ESP32-S3 rev0.2 / 16 MB PSRAM target, and all
three flashed regions passed independent readback verification.  A nine-second
passive check stayed `IDLE`, fresh, camera-error-free, and at `cmd=0,0,0`; no
movement command was sent.

The current unordered-target build always begins standalone ball mode by
searching for the red ball, while independently remembering the first blue
component that becomes stably detected after the run starts.  The two
image-half observations remain available as `LEFT_TARGET` and `RIGHT_TARGET`
for camera diagnostics only; either can become the delivery goal, and no
screen-side label is consulted by motion decisions.  If two targets become
available together, greater stable-frame history, then confidence and matched
area select the preference.  Once selected, nearest-frame continuity retains
the same visible target if it crosses the image center.  Red acquisition still
has priority: seeing blue alone does not leave `BALL_SEARCH_LEFT/RIGHT`; when
red becomes confirmed, the remembered blue target immediately feeds the
existing lateral route alignment, capture, push alignment, and strict
zero-margin three-frame delivery test.  Host regression covers a target first
appearing in the positional-right slot, continued red search with blue alone,
and retention when a later positional-left target has higher confidence.  The
full ESP-IDF 5.4.4 build passes.  The 0x5ecb0-byte application SHA-256 is
`971B162973A0DA6A563241E683F6D76F7F4E1DB488596EA2D0C2F9E3E6AFB06F`.
COM3 was re-identified as the ESP32-S3 rev0.2 / 16 MB PSRAM target; all three
flashed regions passed independent readback verification.  A nine-second
passive check remained `IDLE`, fresh, camera-error-free, and at `cmd=0,0,0`;
no movement command was sent.

## Next Work

1. Mount the camera rigidly, place the car over the competition black line, and
   inspect UART `CAM` center/width/black/threshold telemetry while manually
   moving the unpowered chassis left and right. Tune ROI and thresholds before
   allowing autonomous motion.
2. With the wheels initially raised, verify that camera-derived left/right
   patterns command the expected steering direction; then do a low-speed floor
   test with RESET immediately accessible.
3. Ground-test the enabled retuned left / timed forward / ramped right sequence
   with space to stop it by RESET or serial `x`; capture state transitions and
   measure all three physical displacements.
4. On the competition surface, measure at least five `q/e` displacements and
   five `g` forward displacements. Use medians to calibrate lateral and forward
   milliseconds per centimetre.
5. Measure the scaled 1030/1074 ms fixed segments and the full 1097 ms right
   segment. Verify its 300-to-380, 400 ms acceleration, confirm that all camera
   line patterns are ignored after `BRAKE`, then measure the fixed 525 ms
   straight-forward segment and subsequent latched `FINISHED` stop.
6. Filter the motor-induced encoder glitches before replacing time-calibrated
   segments with encoder distance control. Add MPU6500 only if heading drift
   makes the open-loop rectangular path insufficiently repeatable.
7. Raise the wheels first and use `b` to verify red/blue lateral route-shift
   directions, red/goal search yaw signs, and every stopped observation state
   with `x` immediately available. Then floor-test from 20 cm. Confirm that the
   zero-margin, three-frame red-center-inside-blue-box test stops at the desired
   physical delivery point before trusting the terminal stop.
