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
  search speed and timing remain unchanged. If line loss
  immediately follows reliable cruise motion, the controller keeps the last
  unassisted cruise command for 120 ms to cross the camera's near blind spot,
  commands zero for the remaining 30 ms, then begins an in-place search in the
  last reliable direction. Startup without line/motion history commands zero
  for the full 150 ms; it never uses blind forward motion. Search speed now
  exceeds the 200 drive-assist threshold,
  so search start and every direction reversal use a 500 command for 150 ms
  before settling to 211. Consecutive search legs alternate direction and last
  1200, 2400, 3600, 4800, 6000, and 7200 ms. This reaches equivalent offsets of
  1.2, 2.4, and 3.6 seconds on both sides of the original heading. Later legs
  remain capped at 7200 ms and alternate indefinitely, rather than expanding
  without bound. Search has no total timeout and continues until a connected
  line is found. A
  candidate stops the rotation
  immediately and must remain valid for three distinct decoded camera frames
  before forward line following resumes. Opposite directions require three
  consecutive samples; while unconfirmed, the locked direction is held with
  control-error magnitude at least 4. `0000` rotates in the locked direction,
  defaulting left before a direction has been learned.
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
- `+` and `-` adjust speed.
- USB Host receives 640x480 MJPEG at 15 fps. `esp_jpeg` decodes directly at
  1/8 scale to 80x60, and only the lower half reaches the vision entry point.
  The active native-view ROI uses the central horizontal span
  `x=25%..75%` while retaining `y=60%..93%`. Horizontal coordinates, component
  width, and component area use the calibrated 1x logical scale. The dark
  histogram percentile is 2% for the restored central-width background. The
  adaptive black threshold uses `min(Otsu + contrast/16, 120)` (tightened from
  `Otsu + contrast/8` on 2026-09-01). The absolute 120/255 ceiling prevents a
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
  `heading=far-near` from that component's bottom and top thirds. Continuous
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
  the alternating search. A camera result older than 350 ms during autonomy is
  a camera safety failure: it enters `FAULT` and disables the motors in that
  control cycle.
- Telemetry prints the virtual pattern plus camera freshness, decoded sequence,
  near center (`pos`), far center, heading, steering, width, connected-component
  count/height/area (`comp=count/height/area`), black fraction, threshold,
  contrast, drops, errors, ultrasonic distance, and A/B/C encoder counts. In the virtual pattern, `1` means the
  corresponding camera-derived lane region sees black.
- The same decoded 80x60 RGB888 frame now also feeds a read-only red-ball
  detector in `main/control/camera_ball_vision.c`. It does not influence any
  motor policy. The initial red gates are R>=45, R at least 8 above max(G,B),
  and R/RGB>=380 permille. Eight-connected blobs then need mean R at least 40
  above mean max(G,B), plus at least
  5 permille frame area, 350 permille bounding-box fill, 600 permille
  short/long-side roundness, and a two-pixel image-edge margin. Three frames
  within 200 permille position tolerance confirm a detection. UART `BALL`
  reports color/candidate/detected/stable-count plus position, dimensions,
  shape, confidence, mean RGB, and a raw red probe. The real red ball present
  during the 2026-09-01 first test confirmed continuously near x=-81..-84,
  y=714..717, with mean RGB about 160/97/90 and the raw red probe about
  210/93/80. The final filtered build confirmed the ball near x=+41..+43,
  y=716..717 without jumping to the weak red background; after removal, a
  12-second sample remained `BALL=2/0/0/0`. Ball data is still read-only and
  must not affect motion until a separately reviewed push-ball state machine
  is added.
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
  entering `CLEAR`. In active line following, the 20--60 mm raw-distance
  test is the only ultrasonic condition allowed to interrupt `CLEAR`; even an
  Echo-high or malformed-edge diagnostic does not alter the line policy.
  Automatic bypass
  is enabled for bounded field calibration. The first raw Echo from 20 through
  60 mm stops immediately, then the active sequence is `BRAKE -> LEFT_STRAFE
  -> SETTLE_FORWARD -> FORWARD_19CM -> SETTLE_RIGHT -> RIGHT_12CM ->
  LINE_CONFIRM`. The experimental camera-heading alignment state and all its
  configuration were removed after the live 2026-09-01 test turned once and
  ended in latched `FAILSAFE`. `BRAKE` now transitions directly to left strafe
  after 150 ms. Because encoder interference is not yet filtered, the distance segments
  are scaled open-loop times: left 1170 ms, forward 1191 ms, and right 960 ms.
  Left is about 5% shorter than its preceding 1231 ms setting. Both
  lateral steady/start body commands are restored to 380/500; forward remains
  at 400/500 commands and 1191 ms. The trigger has now returned from 100 to
  60 mm, but the longer forward segment is intentionally retained so the car
  clears about 40 mm farther beyond the obstacle. All segments still require
  ruler calibration.
  The right segment is a 960 ms maximum: its first black sample writes zero in
  the same 20 ms control cycle, then five stationary confirmation cycles precede
  line-follow resume. Once confirmed, the car uses normal camera steering for
  exactly 1000 ms and then enters the latched `FINISHED` stop state. No line by
  the deadline or loss during confirmation stops the open-loop strafe and
  resumes the alternating line search instead of entering `FAILSAFE`.
  Near-obstacle and repeated ultrasonic uncertainty faults remain fail-safe
  stops. `1111` is always treated as an ordinary line pattern and no longer
  triggers finish.
- Ultrasonic authorization requires three consecutive obstacle-free
  observations; either a valid far Echo or clean low-Echo no-return/timeout
  counts, matching the mostly open competition course. Echo-high, malformed,
  or electrically uncertain samples never authorize startup. After entering
  normal `CLEAR` line following,
  every ultrasonic result other than a raw Echo from 20 through 60 mm is
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

The active closer-obstacle / longer-finish build uses raw Echo 20--60 mm to
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

The current installation uses ESP-IDF at `C:\esp\v5.4.4\esp-idf` and Espressif tools under `C:\Espressif\tools`. These are machine-local paths and are not committed as VS Code settings.

## Next Work

1. Mount the camera rigidly, place the car over the competition black line, and
   inspect UART `CAM` center/width/black/threshold telemetry while manually
   moving the unpowered chassis left and right. Tune ROI and thresholds before
   allowing autonomous motion.
2. With the wheels initially raised, verify that camera-derived left/right
   patterns command the expected steering direction; then do a low-speed floor
   test with RESET immediately accessible.
3. Ground-test the enabled shortened left / 19 cm forward / shortened right sequence
   with space to stop it by RESET or serial `x`; capture state transitions and
   measure all three physical displacements.
4. On the competition surface, measure at least five `q/e` displacements and
   five `g` forward displacements. Use medians to calibrate lateral and forward
   milliseconds per centimetre.
5. Measure the scaled 1170/1191 ms fixed segments and the right segment's actual
   early-stop time (960 ms maximum). Verify same-cycle braking, five-cycle line
   confirmation, 1000 ms camera-steered forward motion, and the subsequent
   latched `FINISHED` stop. Confirm that `1111` cannot end that interval early.
6. Filter the motor-induced encoder glitches before replacing time-calibrated
   segments with encoder distance control. Add MPU6500 only if heading drift
   makes the open-loop rectangular path insufficiently repeatable.
