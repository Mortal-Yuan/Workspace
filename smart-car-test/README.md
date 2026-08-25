# Smart Car Test

ESP-IDF 5.4.4 bring-up project for an ESP32-S3 three-wheel omnidirectional smart car.

Current verified features:

- Three PWM motor channels with direction control.
- Three AB encoder inputs.
- Four-channel active-low infrared line sensor input.
- HC-SR04 trigger/echo ranging telemetry.
- USB Serial/JTAG command console.

See [电机与红外测试记录.md](电机与红外测试记录.md) for the confirmed wiring, wheel positions, forward combination, and infrared mapping. See [AGENTS.md](AGENTS.md) for project context, local reference conclusions, known limitations, and handoff notes.

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
| `w` | Move forward using the verified wheel combination |
| `s` | Move backward |
| `x` | Stop all motors |
| `r` | Run a short A/B/C motor test |
| `+` / `-` | Increase/decrease PWM speed |
| `c` | Clear encoder counts |
| `h` | Print command help |

The current `a` and `d` commands have not been calibrated for this three-wheel chassis and should not be treated as valid turn commands yet.

At startup all motors remain stopped. Telemetry prints the four infrared inputs, ultrasonic distance, and A/B/C encoder counts every 500 ms.
