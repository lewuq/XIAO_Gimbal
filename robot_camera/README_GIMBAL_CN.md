# robot_camera

本目录是 XIAO ESP32S3 Sense 的 ESP-IDF 摄像头与人脸追踪工程。

当前实现使用 OV2640、QVGA JPEG 和 `ESPDET_PICO_224_224_FACE`，通过网页显示视频与检测框，并使用 D6/GPIO43 TX、D7/GPIO44 RX 与 STM32C5 双向通信。

完整接线、编译烧录、网页操作、UART 协议、机械限位和故障排查请阅读根目录的 [中文说明](../README_CN.md)；英文说明见 [README.md](../README.md)。

当前功能是人脸检测，不是身份识别。
