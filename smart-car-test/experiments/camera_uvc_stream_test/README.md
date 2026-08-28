# USB 摄像头连续帧实验

在 `camera_usb_probe` 已通过后运行本实验。它使用 Espressif 的
`usb_host_uvc` 2.5.1 组件与板载 16MB 八线 PSRAM，依次尝试：

1. MJPEG 640x480 @ 15 fps；
2. MJPEG 320x240 @ 30 fps；
3. MJPEG 320x240，接受摄像头给出的帧率。

收到 10 个非空视频帧后输出 `CAMERA_STREAM_RESULT=PASS`。整个实验期间
电机使能、PWM 和方向引脚保持低电平，红外寻线任务不会启动。

## 2026-08-28 实机结果

- 摄像头：`VID=349c`、`PID=3307`、产品名 `HD video`；
- USB：Full-Speed，视频端点为 Bulk IN；
- 协商结果：MJPEG 640x480 @ 15 fps；
- 连续接收 10 帧，共 281020 字节，最后一帧 26256 字节；
- 最终结果：`CAMERA_STREAM_RESULT=PASS`。

通过后固件会停止视频流并保持电机锁止。它是实验固件，不会启动小车。
