# Smart Car Project Context

This file is the handoff summary for agents working in `smart-car-test`.

## Project Scope

- Target board: ESP32-S3-DevKitC-1 / ESP32-S3-WROOM-2.
- Framework: ESP-IDF 5.4.4.
- Course platform: three-wheel omnidirectional smart car.
- Current phase: bring-up and calibration of motors, encoders, infrared line sensors, and ultrasonic ranging.
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
- Infrared physical order, left to right: `OUT4/GPIO5`, `OUT3/GPIO4`, `OUT2/GPIO2`, `OUT1/GPIO1`.
- Infrared outputs are active low: black/indicator on is `0`; white/indicator off is `1`.
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
- Manual motion and the adjustable line-speed ceiling default to `450/1000`.
  Line following uses `420/1000` on
  `0110`/`1111`, `310/1000` on ordinary curves, and `280/1000` on one-sided
  edge patterns. Curve commands are capped at `500/1000` or `450/1000`, and
  nonzero curve-wheel commands have a `220/1000` minimum after scaling.
- Line-follow motors A/C use an encoder-free starting boost: on initial motion
  or a direction reversal, a requested magnitude below `500/1000` is raised to
  `500/1000` for 150 ms, then returns to the controller request. A true zero for
  120 ms rearms a same-direction boost. Avoidance/manual commands bypass this
  helper. Do not add encoder speed control until motor-induced encoder glitches
  are filtered or electrically corrected.
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
- Line following normalizes the active-low inputs and computes a proportional error using scaled physical left-to-right weights `-6, -2, +2, +6`.
- It steers with motors A/C around the confirmed forward combination and keeps
  rear motor B stopped because B direction is not yet calibrated. Search
  direction is independently latched; for `0000`, equal-and-opposite A/C search
  runs at `360/1000`.
- Latest physical-test result: after broader testing, the direction latch, three-cycle confirmation, and edge-speed limit handle the great majority of sharp corners sufficiently well. Rare direction-dependent entry cases remain an optional optimization point, not a required fix or blocker. Preserve both the earlier poor test and this newer conclusion in future handoffs.
- Telemetry prints normalized detections in physical `L,LC,RC,R` order, ultrasonic distance, and A/B/C encoder counts every 500 ms. In this display, `1` means black detected.
- Follow-mode logs include timestamp, control state, infrared pattern and stability count, active sensor count, current/last error, selected base speed, ultrasonic distance, obstacle state, limited A/B/C commands, and encoder counts.
- HC-SR04 samples have `VALID/OUTLIER/LOST` quality, actual Echo edge-level
  validation, a three-sample median, and jump confirmation. Automatic bypass
  is enabled for bounded field calibration. The first raw Echo from 20 through
  200 mm stops immediately, then the active rectangular sequence is
  `BRAKE -> LEFT_EDGE -> LEFT_CLEARANCE -> FORWARD_BYPASS -> RIGHT_LINE ->
  LINE_CONFIRM`, with settle states and fail-safe timeouts. Its 800 ms lateral
  clearance and 2500 ms forward duration are provisional field parameters, not
  yet measured 10 cm and 40 cm distances.
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
  B positive has a 460 minimum; pure lateral A/C have a 300 minimum; lateral
  yaw compensation is 20%. Automatic bypass is enabled for bounded field
  calibration after `q/e` confirmed physical direction. `q/e` run for 1000 ms; `g` runs the verified
  forward basis at 400/1000 for 1000 ms for ruler calibration.

## Build and Flash

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
  --flash_mode dio --flash_freq 80m --flash_size 2MB `
  0x0 build/bootloader/bootloader.bin `
  0x8000 build/partition_table/partition-table.bin `
  0x10000 build/smart-car-test.bin
```

The current installation uses ESP-IDF at `C:\esp\v5.4.4\esp-idf` and Espressif tools under `C:\Espressif\tools`. These are machine-local paths and are not committed as VS Code settings.

## Next Work

1. Field-test the enabled rectangular bypass with space to stop it by RESET or
   serial `x`; capture state transitions and physical displacement.
2. On the competition surface, measure at least five `q/e` displacements and
   five `g` forward displacements. Use medians to calibrate the 10 cm lateral
   clearance and 40 cm forward durations.
3. Replace the provisional 800/2500 ms values with the measured 10/40 cm
   medians. The first line detection already brakes before a five-cycle
   stationary confirmation; edge and line searches have 5 s limits.
4. Filter the motor-induced encoder glitches before replacing time-calibrated
   segments with encoder distance control. Add MPU6500 only if heading drift
   makes the open-loop rectangular path insufficiently repeatable.
