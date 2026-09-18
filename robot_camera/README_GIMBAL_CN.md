# XIAO ESP32S3 Sense + STM32C5 人像追踪云台

## 功能

- OV2640 QVGA JPEG 采集与 Wi-Fi MJPEG 视频流
- ESP-DL YOLO26n INT8（COCO `person` 类）人像追踪
- 网页 AUTO/MANUAL/START 控制和触摸摇杆
- D6/D7 UART 以 CRC16 定长帧发送 yaw/pitch 目标到 STM32C5

## 接线

| ESP32S3 Sense | STM32C5 | 说明 |
|---|---|---|
| D6 / GPIO43 TX | D7 / PA10 RX | 控制数据 |
| D7 / GPIO44 RX | D6 / PA9 TX | 预留回传 |
| GND | GND | 必须共地 |

STM32C5 圆屏背光 PWM 已移除，D6/D7 完整恢复为 USART1。背光由硬件控制。

## 使用

1. 分别在两个工程目录运行 `pio run -t upload`。
2. 上电后先确保电机/CAN 正常，再连接 Wi-Fi：`Gimbal-ReCamera`，密码 `gimbal123`。
3. 浏览器打开 `http://192.168.4.1`，点击 `START` 完成原工程的安全回中。
4. `AUTO` 使用最大 person 框中心追踪；`MANUAL` 后拖动摇杆控制。

## 参数

- UART：115200 8N1，ESP32S3 D6/D7。
- 摄像头：OV2640，QVGA，JPEG quality 12，双 framebuffer。
- 追踪死区：画面宽高的 8%；单次最大 yaw/pitch 增量约 4°/3°。
- STM32C5 仍执行已有机械限位、CAN 反馈超时和故障保护。

## 构建说明

工程使用 ESP-IDF，通过 Component Manager 获取 `espressif/esp32-camera` 与 `espressif/esp-dl ~3.3.0`。`components/yolo26` 和 ESP32-S3 的 512x512 INT8 模型已随工程保存。

当前 SeeedStudio PlatformIO 包的 ESP-IDF 构建文件和工具链声明不完整，因此本工程使用已安装的 `espressif32@55.3.311`、ESP-IDF 5.5.5，并在 `boards/` 中保存 XIAO Sense 板卡定义。推荐构建命令：

```powershell
.\build_espidf.ps1
```

验证结果：`build_idf/gimbal_recamera.bin` 大小约 5.62 MiB，7 MiB app 分区剩余约 20%。

烧录（将 `COM8` 换成设备管理器中实际串口）：

```powershell
.\flash_espidf.ps1 -Port COM8
```

烧录后立即打开串口监视器：

```powershell
.\flash_espidf.ps1 -Port COM8 -Monitor
```

如果自动复位失败，按住 XIAO 的 BOOT，点按 RESET，松开 BOOT 后重新执行烧录命令。
