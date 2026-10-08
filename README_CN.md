# XIAO 双轴人脸追踪云台

[English](README.md) | [中文](README_CN.md)

这是一个完全在设备端运行的双轴人脸追踪云台，由 **Seeed Studio XIAO ESP32S3 Sense**、**XIAO STM32C5** 和两台 LK 系列 CAN 电机组成。ESP32-S3 负责 OV2640 图像采集、ESP-DL 人脸检测、Wi-Fi 网页和 UART 目标角度发送；STM32C5 通过 Classic CAN 执行电机控制，并把实时角度回传给 ESP32-S3。

> 本项目属于实验性 Maker 工程，不是经过认证的稳定或安全系统。调试时请让手、线缆远离运动结构，并首先使用保守限位进行测试。

## 功能

- OV2640 QVGA JPEG 图像采集和 Wi-Fi MJPEG 视频流
- ESP32-S3 本地运行 ESPDet-Pico 224 × 224 INT8 人脸检测
- 自动调节 yaw/pitch，使画面中最大的人脸保持在中央附近
- 网页显示视频、检测框、AUTO/MAN、虚拟摇杆、角度滑条、CENTER 和 CAL
- 115200 8N1 全双工 UART，使用 CRC-16/CCITT 定长协议
- STM32C5 电机控制器，D6/D7 专用于 USART1
- Yaw：MS3506，CAN ID `0x141`
- Pitch：MS3008，CAN ID `0x142`
- 1 Mbit/s Classic CAN、编码器反馈、机械限位、通信超时和故障状态
- STM32C5 以 20 Hz 回传电机角度并显示在网页中

## 系统结构

```text
OV2640 摄像头
      │ QVGA JPEG
      ▼
XIAO ESP32S3 Sense
  ├─ ESP-DL 人脸检测
  ├─ Wi-Fi AP + 控制网页
  └─ UART D6 TX / D7 RX
             │
             ▼
XIAO STM32C5
  ├─ UART 协议解析与遥测
  ├─ 限位和安全状态机
  └─ 1 Mbit/s Classic CAN
       ├─ 0x141 MS3506 Yaw
       └─ 0x142 MS3008 Pitch
```

## 仓库目录

```text
├─ robot_camera/   ESP-IDF 摄像头、人脸检测、网页和 UART 工程
├─ gimbal_motor/   Zephyr STM32C5 UART/CAN 电机工程
├─ README.md
└─ README_CN.md
```

## 硬件

- XIAO ESP32S3 Sense，使用 **OV2640** 摄像头扩展板
- XIAO STM32C5
- Yaw 轴 LK MS3506，节点 1 / CAN ID `0x141`
- Pitch 轴 LK MS3008，节点 2 / CAN ID `0x142`
- 符合电机要求的独立电源
- 正确终端匹配的 Classic CAN 总线
- 两块 XIAO 与电机/CAN 电源系统必须共地

不要使用 XIAO 的 3.3 V 或 5 V 引脚给电机供电。

## 接线

| XIAO ESP32S3 Sense | XIAO STM32C5 | 用途 |
|---|---|---|
| D6 / GPIO43 TX | D7 / PA10 RX | 向 STM32C5 发送命令 |
| D7 / GPIO44 RX | D6 / PA9 TX | 向 ESP32-S3 回传角度 |
| GND | GND | 信号共地 |

TX 与 RX 必须交叉连接。两端均为 `115200 8N1`。将 STM32C5 的 CANH、CANL、GND 与两台电机连接成一条总线，通常在两个物理末端各使用一个 120 Ω 终端电阻。固件通过 `PB14` 控制 CAN 收发器待机引脚。不能把 UART 直接接到 CANH/CANL。

STM32C5 圆屏背光 PWM 已移除，D6/D7 完整恢复为 USART1。背光由硬件控制。

## 软件环境

STM32C5 使用 PlatformIO、Seeed Studio 平台和 Zephyr 4.4.0。ESP32-S3 使用 ESP-IDF 5.5.x；仓库 Windows 脚本依赖 PlatformIO Core、`espressif32@55.3.311`、Python 3.11 和 PowerShell。`robot_camera/dependencies.lock` 已固定 ESP-DL 3.3.11、esp32-camera 2.1.7 等依赖。

## 编译与烧录

```bash
git clone https://github.com/lewuq/XIAO_Gimbal.git
cd XIAO_Gimbal
pio run -d gimbal_motor
pio run -d gimbal_motor -t upload
```

STM32C5 UF2 位于 `gimbal_motor/.pio/build/seeed-xiao-stm32c5/firmware.uf2`。

ESP32-S3 常规 ESP-IDF：

```bash
cd robot_camera
idf.py set-target esp32s3
idf.py build
idf.py -p COM8 flash monitor
```

已验证的 Windows/PlatformIO 辅助方式：

```powershell
cd robot_camera
pio pkg install
.\build_espidf.ps1
.\flash_espidf.ps1 -Port COM8 -Monitor
```

请替换实际串口。如果无法进入下载模式，请按住 **BOOT**，短按 **RESET**，松开 **BOOT** 后重试。构建目录、`sdkconfig`、managed components 和 IDE 文件均由 `.gitignore` 排除。

## 首次运行与网页控制

1. 检查接线时先卸下机械负载。
2. 给 STM32C5 和电机总线上电，确认 `0x141`、`0x142` 均有反馈。
3. 给 ESP32-S3 上电，确认 UART 已交叉且共地。
4. 连接 Wi-Fi `Gimbal-ReCamera`，密码 `gimbal123`。
5. 打开 `http://192.168.4.1`。
6. 仅在编码器映射和机械结构安全时使用 **CAL**。
7. **CENTER** 回到 yaw `180°`、pitch `90°`；**AUTO** 启用人脸追踪；**MAN**、滑条和摇杆用于手动控制。

控制接口使用 80 端口，MJPEG 使用 81 端口，视频重连不会阻塞控制请求。

## 默认参数

| 参数 | 数值 |
|---|---:|
| Yaw 硬限位 / 控制范围 | `0…345°` / `1…344°` |
| Pitch 硬限位 / 控制范围 | `0…180°` / `1…175°` |
| 回中目标 | yaw `180°`、pitch `90°` |
| 回中 / 普通速度 | `20°/s` / `30°/s` |
| 人脸阈值 | `0.30` |
| 摄像头 | QVGA JPEG、质量 12、双 framebuffer |
| UART 遥测 | 20 Hz |

`gimbal_motor/include/app_config.h` 中的编码器映射来自一套样机实测。改变电机、减速箱、编码器零位或机械安装后必须重新标定。

## UART 协议

多字节字段为小端序；角度为无符号百分之一度；CRC 为 CRC-16/CCITT-FALSE，多项式 `0x1021`，初始值 `0xFFFF`。

ESP32-S3 → STM32C5 为 12 字节：`AA 55`、版本、标志、序号、yaw、pitch、CRC。标志 bit 0=`CENTER`、bit 1=`AUTO`、bit 2=`FACE_VALID`、bit 3=`CAL`。

STM32C5 → ESP32-S3 为 16 字节：`55 AA`、版本、云台状态、序号、yaw 实测、pitch 实测、yaw 目标、pitch 目标、CRC。

## 故障排查

- **有视频但电机不动：**检查 UART 是否交叉、是否共地、STM32 UART `valid` 是否增长、状态是否为 READY，以及 CAN 速率和 ID。
- **AUTO 比手动慢：**检查 `FACE detections=0`，确认 STM32 使用 `0.25°` 目标死区，并观察 `AUTO TRACK ex=... ey=... target=... motor=...`。
- **追踪把人脸推向边缘：**摄像头镜像或电机安装方向不同，需要在 `robot_camera/src/main.cpp` 调整方向符号。
- **复制到非 Git 目录后构建失败：**工程已声明 `PROJECT_VER`；只删除自动生成的 `robot_camera/build_idf` 后重建。

## 已知限制

- 当前是人脸检测，不是身份识别。
- 多张人脸时选择最大检测框。
- 延迟取决于推理、JPEG 解码、Wi-Fi 和电机响应。
- 标定参数和方向与机械装配相关。
- Wi-Fi 凭据编译在固件中，部署前应修改。
- Windows 脚本依赖 PlatformIO 管理的 ESP-IDF 5.5.5。

## 许可证

项目原创代码采用 [MIT 许可证](LICENSE)。第三方组件和模型仍遵循各自的许可证。

## 致谢

[Seeed Studio](https://wiki.seeedstudio.com/xiao_esp32s3_getting_started/) · [ESP-IDF](https://github.com/espressif/esp-idf) · [ESP-DL](https://github.com/espressif/esp-dl) · [PlatformIO](https://platformio.org/) · [Zephyr](https://www.zephyrproject.org/)
