# Smart Car Project Context

This file is the handoff summary for agents working in `smart-car-test`.

## Project Scope

- Target board: ESP32-S3-DevKitC-1 / ESP32-S3-WROOM-2.
- Framework: ESP-IDF 5.4.4.
- Course platform: three-wheel omnidirectional smart car.
- Current phase: replacing the four-channel infrared sensor with a USB UVC
  camera, then calibrating camera line following and ultrasonic obstacle bypass.
- Course direction from the lecture: line following and obstacle avoidance first, with camera/ball interaction and optional extensions in later phases.

The detailed, experimentally confirmed motor and infrared mapping is in [电机与红外测试记录.md](电机与红外测试记录.md). The complete development history, wiring summary, algorithm experiments, failures, and latest physical-test conclusions are in [项目开发与实验日志.md](项目开发与实验日志.md). Treat those documents and the current source as authoritative when older notes disagree.

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
- The current uncommitted UART0 diagnostic uses `TX=GPIO43` and `RX=GPIO44`
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
  applies `correction=clamp(error*100,-250,250)` to
  `A=-base+correction, B=0, C=-base-correction`.
- Camera bring-up line bases/limits are straight 240, ordinary curve 190/320,
  one-sided edge 160/280, and no-line search 240. Opposite directions require three
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
  1/8 scale to 80x60; the lower 60%--93% of the native view is thresholded
  adaptively and its largest black column component is mapped to virtual
  `L/LC/RC/R`. Board testing sustained about 13.6 decoded frames/s with no
  decode error or watchdog warning after adding a one-tick yield per frame.
- Line following consumes the virtual `L,LC,RC,R`, actively steers with A/C
  differential correction, and searches after line loss. Place the car on the
  line before BOOT. BOOT/`f` is ignored until a fresh camera result exists; a
  camera result older than 350 ms during autonomy enters `FAULT` and disables
  the motors in that control cycle.
- Telemetry prints the virtual pattern plus camera freshness, decoded sequence,
  center, width, black fraction, threshold, contrast, drops, errors, ultrasonic
  distance, and A/B/C encoder counts. In the virtual pattern, `1` means the
  corresponding camera-derived lane region sees black.
- Camera vision analyzes the native view's central near-track window
  (`x=25%..75%`, `y=60%..93%`). Horizontal center and component width are
  normalized to that window, so dark laboratory background outside it cannot
  dominate the actual line. The remounted camera uses a +37 permille center
  offset from the median of 35 centered, sub-pixel telemetry samples. The
  weighted centroid retains fractional-column precision before normalization.
  A final 35-sample run reported median center 0 and range 0..5, always `0110`.
- The cropped window is divided into five equal steering bands at -600, -200,
  +200, and +600 permille. The wider center band avoids one-pixel quantization
  jitter toggling a physically centered car between straight and curve.
- Follow-mode logs publish the raw weighted error, direction-validated control
  error, selected base speed, pattern state, and final A/B/C command.
- HC-SR04 samples have `VALID/OUTLIER/LOST/INVALID/NO_RETURN` quality, actual
  Echo edge-level validation, a three-sample median, and jump confirmation.
  Ranging now uses a nominal 45 ms deadline and a 70 ms trigger period. A
  completed pulse above 4000 mm is reported as `NO_RETURN` (`q=5`). Once
  startup has been authorized, completed no-return pulses, low-Echo timeouts,
  and clean far-distance `OUTLIER` jumps are treated as normal open space and
  do not leave `CLEAR`. In active line following, the 20--100 mm raw-distance
  test is the only ultrasonic condition allowed to interrupt `CLEAR`; even an
  Echo-high or malformed-edge diagnostic does not alter the line policy.
  Automatic bypass
  is enabled for bounded field calibration. The first raw Echo from 20 through
  100 mm stops immediately, then the active sequence is `BRAKE -> LEFT_15CM ->
  SETTLE_FORWARD -> FORWARD_19CM -> SETTLE_RIGHT -> RIGHT_12CM ->
  LINE_CONFIRM`. Because encoder interference is not yet filtered, these are
  scaled open-loop time segments: left 1231 ms, forward 1191 ms, and right
  960 ms. Left is another 5% shorter than its preceding 1296 ms setting. Both
  lateral steady/start body commands are restored to 380/500; forward remains
  at 400/500 commands and 1191 ms. The forward segment was lengthened by the
  nominal 40 mm made necessary when the stop threshold moved from 60 to 100 mm.
  All segments still require ruler calibration.
  The right segment is a 960 ms maximum: its first black sample writes zero in
  the same 20 ms control cycle, then five stationary confirmation cycles precede
  line-follow resume. No line by the deadline or loss during confirmation enters
  `FAILSAFE`. Before a bypass completes, `1111` remains an ordinary line pattern;
  afterward it enters a separate latched `FINISHED` stop state.
- Ultrasonic authorization is fail-safe: startup requires three consecutive
  valid far-Echo observations. After entering normal `CLEAR` line following,
  every ultrasonic result other than a raw Echo from 20 through 100 mm is
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
  compensation remains direction-specific but both sides are currently 50%.
  At the 380 steady lateral command this produces
  `A/B/C=-300/-570/+300` left and `+300/+570/-300` right; the 500 start
  command produces `-300/-750/+300` and `+300/+750/-300`. Automatic bypass is enabled for bounded field
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
5. Measure the scaled 1231/1191 ms fixed segments and the right segment's actual
   early-stop time (960 ms maximum). Verify same-cycle braking, five-cycle line
   confirmation, line-follow resume, and the post-bypass `1111 -> FINISHED` stop.
6. Filter the motor-induced encoder glitches before replacing time-calibrated
   segments with encoder distance control. Add MPU6500 only if heading drift
   makes the open-loop rectangular path insufficiently repeatable.
