# XIAO Dual-Axis Face-Tracking Gimbal

[English](README.md) | [中文](README_CN.md)

An on-device dual-axis face-tracking camera built with a **Seeed Studio XIAO ESP32S3 Sense**, a **XIAO STM32C5**, and two LK-series CAN motors. The ESP32-S3 captures OV2640 video, runs ESP-DL face detection, serves a browser control panel, and sends target angles over UART. The STM32C5 controls both motors over Classic CAN and returns live angle telemetry.

> This is an experimental maker project, not a certified safety or stabilization system. Keep hands and cables away from the mechanism and test with conservative limits first.

## Features

- OV2640 QVGA JPEG capture and Wi-Fi MJPEG streaming
- ESPDet-Pico 224 × 224 INT8 face detection on the ESP32-S3
- Automatic yaw/pitch tracking of the largest detected face
- Browser UI with live video, detection box, AUTO/MAN, joystick, angle sliders, CENTER, and CAL
- Full-duplex 115200 8N1 UART with CRC-16/CCITT packets
- STM32C5 motor controller with D6/D7 dedicated to USART1
- MS3506 yaw motor at CAN ID `0x141`
- MS3008 pitch motor at CAN ID `0x142`
- Classic CAN at 1 Mbit/s, encoder feedback, limits, timeouts, and fault reporting
- 20 Hz motor-angle telemetry displayed in the browser

## Architecture

```text
OV2640 camera
      │ QVGA JPEG
      ▼
XIAO ESP32S3 Sense
  ├─ ESP-DL face detector
  ├─ Wi-Fi AP + browser UI
  └─ UART D6 TX / D7 RX
             │
             ▼
XIAO STM32C5
  ├─ UART parser + telemetry
  ├─ safety/limit state machine
  └─ Classic CAN 1 Mbit/s
       ├─ 0x141 MS3506 yaw
       └─ 0x142 MS3008 pitch
```

## Repository Layout

```text
├─ robot_camera/   ESP-IDF camera, detection, Wi-Fi UI, and UART firmware
├─ gimbal_motor/   Zephyr STM32C5 UART/CAN motor firmware
├─ README.md
└─ README_CN.md
```

## Hardware

- XIAO ESP32S3 Sense with the **OV2640** camera board
- XIAO STM32C5
- LK MS3506 yaw motor, node 1 / CAN ID `0x141`
- LK MS3008 pitch motor, node 2 / CAN ID `0x142`
- A suitable external motor power supply
- Correctly terminated Classic CAN bus
- Common ground between both XIAO boards and the motor/CAN system

Do not power either motor from a XIAO 3.3 V or 5 V pin.

## Wiring

### ESP32-S3 ↔ STM32C5 UART

| XIAO ESP32S3 Sense | XIAO STM32C5 | Purpose |
|---|---|---|
| D6 / GPIO43 TX | D7 / PA10 RX | Commands to STM32C5 |
| D7 / GPIO44 RX | D6 / PA9 TX | Telemetry to ESP32-S3 |
| GND | GND | Common signal ground |

TX and RX must be crossed. Both ends use `115200 8N1` without flow control.

The STM32C5 firmware no longer controls the round-display backlight through PWM. D6/D7 are fully assigned to USART1; backlight control, when present in the hardware assembly, is handled in hardware.

### STM32C5 ↔ CAN Motors

Connect CANH, CANL, and ground to both motors as one bus. A typical bus uses 120 Ω termination at each physical end. The firmware controls transceiver standby on `PB14`. Do not connect UART pins directly to CANH/CANL.

## Software Requirements

### STM32C5

- PlatformIO Core or VS Code with PlatformIO
- Seeed Studio platform providing `seeed-xiao-stm32c5`
- Zephyr 4.4.0 package selected by that platform

### ESP32-S3

- ESP-IDF 5.5.x
- PlatformIO Core with `espressif32@55.3.311` for the supplied Windows scripts
- Python 3.11 and PowerShell for the tested Windows workflow

Component versions are pinned in `robot_camera/dependencies.lock`, including ESP-DL 3.3.11 and esp32-camera 2.1.7.

## Build and Flash

```bash
git clone https://github.com/lewuq/XIAO_Gimbal.git
cd XIAO_Gimbal
```

### STM32C5

```bash
pio run -d gimbal_motor
pio run -d gimbal_motor -t upload
```

The UF2 output is `gimbal_motor/.pio/build/seeed-xiao-stm32c5/firmware.uf2` and can also be copied through the board bootloader.

### ESP32-S3 with ESP-IDF

```bash
cd robot_camera
idf.py set-target esp32s3
idf.py build
idf.py -p COM8 flash monitor
```

Tested Windows/PlatformIO helper workflow:

```powershell
cd robot_camera
pio pkg install
.\build_espidf.ps1
.\flash_espidf.ps1 -Port COM8 -Monitor
```

Replace `COM8` with the actual port. If download mode fails, hold **BOOT**, tap **RESET**, release **BOOT**, and retry. Generated build directories, `sdkconfig`, managed components, and IDE files are intentionally ignored by Git.

## First Run

1. Mechanically unload or disconnect the motors while checking wiring.
2. Power the STM32C5 and motor bus; confirm both CAN IDs respond.
3. Power the ESP32-S3; confirm crossed UART wiring and common ground.
4. Connect to `Gimbal-ReCamera` with password `gimbal123`.
5. Open `http://192.168.4.1`.
6. Use **CAL** only after verifying the encoder mapping and mechanism.
7. Use **CENTER** to command yaw `180°`, pitch `90°`.
8. Select **AUTO** and place a face in view.

Control/status uses port 80. MJPEG uses port 81 so stream reconnects cannot block controls.

## Browser Controls

- **AUTO** enables face tracking.
- **MAN** disables automatic tracking.
- **CENTER** commands the mechanical home target.
- **CAL** requests the STM32C5 calibration/start sequence.
- Sliders send absolute angles and enter manual mode.
- The joystick sends incremental angles and enters manual mode.

## Limits and Defaults

| Item | Value |
|---|---:|
| Yaw hard / control range | `0…345°` / `1…344°` |
| Pitch hard / control range | `0…180°` / `1…175°` |
| Center | yaw `180°`, pitch `90°` |
| Home / normal speed | `20°/s` / `30°/s` |
| Face threshold | `0.30` |
| Camera | QVGA JPEG, quality 12, two frame buffers |
| UART telemetry | 20 Hz |

Encoder mappings in `gimbal_motor/include/app_config.h` were measured for one assembly. Recalibrate after changing a motor, gearbox, encoder zero, or mechanical installation.

## UART Protocol

Multi-byte fields are little-endian. Angles are unsigned centidegrees. CRC is CRC-16/CCITT-FALSE over all preceding bytes, polynomial `0x1021`, initial value `0xFFFF`.

### ESP32-S3 → STM32C5 command (12 bytes)

| Offset | Type | Field |
|---:|---|---|
| 0 | `u8` | `0xAA` |
| 1 | `u8` | `0x55` |
| 2 | `u8` | version `1` |
| 3 | `u8` | flags |
| 4 | `u16` | sequence |
| 6 | `u16` | yaw centidegrees |
| 8 | `u16` | pitch centidegrees |
| 10 | `u16` | CRC |

Flags: bit 0 `CENTER`, bit 1 `AUTO`, bit 2 `FACE_VALID`, bit 3 `CAL`.

### STM32C5 → ESP32-S3 telemetry (16 bytes)

| Offset | Type | Field |
|---:|---|---|
| 0..3 | `u8` | `55 AA 01`, then gimbal state |
| 4 | `u16` | sequence |
| 6 | `u16` | measured yaw |
| 8 | `u16` | measured pitch |
| 10 | `u16` | yaw target |
| 12 | `u16` | pitch target |
| 14 | `u16` | CRC |

## Troubleshooting

**Video works but motors do not move:** verify crossed UART, common ground, increasing STM32 UART `valid` count, READY state, CAN bitrate, and IDs `0x141/0x142`.

**AUTO is slower than manual:** look for `FACE detections=0`, confirm the STM32 firmware has the `0.25°` target deadband, and inspect `AUTO TRACK ex=... ey=... target=... motor=...`.

**Tracking drives the face toward an edge:** camera mirroring or motor mounting differs from the tested assembly. Adjust the corresponding direction sign in `robot_camera/src/main.cpp`.

**Build from a copied non-Git directory fails:** the project declares `PROJECT_VER`. Remove only generated `robot_camera/build_idf` and rebuild if an older CMake cache remains.

## Known Limitations

- This is face detection, not identity recognition.
- With several faces, the largest box is selected.
- Tracking latency depends on inference, JPEG decode, Wi-Fi load, and motor response.
- Calibration and direction signs are assembly-specific.
- Wi-Fi credentials are compiled into the firmware and should be changed for deployment.
- Windows helpers assume a PlatformIO-managed ESP-IDF 5.5.5 environment.

## License

No repository-level license has been selected. Until a root `LICENSE` is added, normal copyright law applies to original project code. Third-party components and models retain their own licenses. Add a project license before inviting reuse or contributions.

## Acknowledgements

[Seeed Studio](https://wiki.seeedstudio.com/xiao_esp32s3_getting_started/) · [ESP-IDF](https://github.com/espressif/esp-idf) · [ESP-DL](https://github.com/espressif/esp-dl) · [PlatformIO](https://platformio.org/) · [Zephyr](https://www.zephyrproject.org/)
