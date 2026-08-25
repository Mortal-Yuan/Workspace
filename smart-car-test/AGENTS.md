# Smart Car Project Context

This file is the handoff summary for agents working in `smart-car-test`.

## Project Scope

- Target board: ESP32-S3-DevKitC-1 / ESP32-S3-WROOM-2.
- Framework: ESP-IDF 5.4.4.
- Course platform: three-wheel omnidirectional smart car.
- Current phase: bring-up and calibration of motors, encoders, infrared line sensors, and ultrasonic ranging.
- Course direction from the lecture: line following and obstacle avoidance first, with camera/ball interaction and optional extensions in later phases.

The detailed, experimentally confirmed motor and infrared mapping is in [电机与红外测试记录.md](电机与红外测试记录.md). Treat that document and the current source as authoritative when older notes disagree.

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
- HC-SR04 is currently assigned `Trig=GPIO6`, `Echo=GPIO13`. Telemetry has returned distance values, but ranging accuracy and the physical Echo level interface have not yet been formally validated.

Important allocation changes from early notes:

- GPIO41/GPIO42 are now used by motor C encoder E3A/E3B, so they are not available for the earlier proposed MPU6500 I2C connection.
- GPIO14 is now motor B `BIN1`, so the earlier optional DHT11 GPIO14 proposal is invalid.
- Early two-wheel documents label A/B as left/right. Actual tests supersede that: A=right, B=rear, C=left.

## Source Behavior

Main source: `main/main.c`.

- Motors are stopped at startup.
- Default speed is `400/1000` (about 40% duty).
- `1`, `2`, `3` run A/right, B/rear, C/left individually.
- `w`, `s`, `x` are confirmed forward, backward, and stop.
- `r` runs a short A/B/C sequence.
- `+` and `-` adjust speed.
- Telemetry prints infrared raw bits in `OUT1 OUT2 OUT3 OUT4` order, ultrasonic distance, and A/B/C encoder counts every 500 ms.
- The current `a` and `d` implementations are inherited from the old two-wheel assumption and are not valid three-wheel turn commands. Do not use them until omnidirectional motion is calibrated.

## Build and Flash

Normal ESP-IDF workflow:

```powershell
idf.py set-target esp32s3
idf.py build
idf.py -p COM5 flash monitor
```

On the current machine, the board is normally on `COM5`. Standard flashing has previously timed out while uploading the stub. The reliable fallback is:

```powershell
python -m esptool --chip esp32s3 -p COM5 -b 115200 `
  --before default_reset --after hard_reset --no-stub write_flash `
  --flash_mode dio --flash_freq 80m --flash_size 2MB `
  0x0 build/bootloader/bootloader.bin `
  0x8000 build/partition_table/partition-table.bin `
  0x10000 build/smart-car-test.bin
```

The current installation uses ESP-IDF at `C:\esp\v5.4.4\esp-idf` and Espressif tools under `C:\Espressif\tools`. These are machine-local paths and are not committed as VS Code settings.

## Next Work

1. Calibrate valid three-wheel left/right rotation and lateral movement combinations.
2. Rename infrared definitions in code from numeric channel names to physical `LEFT`, `LEFT_CENTER`, `RIGHT_CENTER`, `RIGHT` names.
3. Validate ultrasonic distance against known distances and document the actual Echo electrical connection.
4. Build line-following control using the confirmed active-low infrared order.
5. Add encoder-based speed balancing after motion directions are fully calibrated.
