# 深度点云与每五秒彩色照片试采

## 已验证结果

2026-10-30，机器人 192.168.1.123，主机 mi-desktop。采用现有 SSH 身份与严格主机校验。
证据目录：`evidence/rgbd_samples/20261030_170817_170642/`。

- 约 16 秒静止订阅，收到 422 条彩色图消息、186 条深度图消息。
- 保存 4 张 640×480 彩色 PNG（原始编码 bgr8），照片间隔约 5 秒。
- 保存 4 份 640×480 深度 NPY（16UC1，毫米转换为米）。
- 根据同坐标系、同尺寸、时间差不超过 0.2 秒的深度 CameraInfo，生成 4 份相机坐标系 PLY；每四像素采样一次，每份约 1.67 万个有效点。
- 0.15–4 米范围内有效深度像素比例约 87.1%–87.3%，该比例不代表测距精度。
- 实际查看第一张照片：可见椅子和房间，低机位没有充分看到座面上表面。
- 本次无运动命令；试采已结束，采集器没有关闭原厂图像发布。

## 原厂接口与变更边界

深度节点原状态为 inactive [2]，任务 status=101/code=0，对齐节点 unconfigured [1]。通过原厂 lifecycle activate 激活深度相机，返回成功。
彩色相机 GET_STATE（command=4）返回 result=0；按照本地原厂 developer_guide 文档使用 CameraService command=9、width=640、height=480、fps=0 开启图像发布，返回 result=0。
实际彩色话题是 `/mi_desktop_48_b0_2d_7b_0b_4a/image`；深度话题是同命名空间下 `/camera/depth/image_rect_raw`。
没有更改机器人源代码、控制库、步态或持久配置，没有启动导航或建图任务。

## 工具

- `tools/rgbd_sample_remote.py`：临时内存执行的只读订阅程序，每路每五秒保存一次样本，不在机器人安装文件。
- `tools/capture_rgbd_samples.py`：电脑端保存 PNG、NPY、PLY 与元数据；默认 16 秒，最多 120 秒能力验证。
- 运行：`D:\cyberdog\tools\reconstruction_runtime\.venv\Scripts\python.exe tools/capture_rgbd_samples.py --seconds 16`，工作目录为本项目根目录。必须先确认原厂图像发布已经启用；此工具不会自动启用相机。
- PNG 使用标准库写入，不依赖 Pillow。人工合成输入验证了 BGR 转 RGB、行填充、PNG 校验与像素内容，以及毫米深度到米点云的投影；合成数据单独位于 `evidence/rgbd_codec_synthetic_test/`。

## 未完成事项

这不是同步 RGB-D 记录：两路独立计时保存，已保存帧存在不同时间偏差；详见 timing_analysis.json。
本次未获取彩色相机 CameraInfo、彩色与深度外参或世界位姿，不能直接将照片贴到这些点云上，也不能据此拼接整圈模型。
下一步先获取彩色内外参与对应时间的深度/位姿，再接入正式采集网页。每五秒保存照片可用于视角参考；移动时用于几何配准的图像频率应根据重叠和运动速度另行确定。
本次没有进行多方位移动测试。后续移动由用户控制，采集程序不能让机器人停止运动。

## 后续静止时间匹配验证

记录目录：`evidence/rgbd_samples/20261030_171129_366554/`。新增 `--paired` 选项，在每五秒选取照片后等待 0.15 秒，从最多 20 个深度帧的缓存中选择时间最近者，超过 50 毫秒则拒绝保存该对。此阈值是采集验收门槛，并非高速运动下精度保证。

实测保存 4 对彩色/深度，0 对因时间差被拒绝；逐对核对时间差为 22.779、9.356、11.527、4.461 毫秒，每对均生成照片与有效深度点云。
该结果是软件时间戳匹配通过，不代表硬件同步或空间配准通过。

通过严格 SSH 只读保存了 `/params/camera/calibration/params_intrinsic.yaml`、`params_extrinsic.yaml`，以及原厂 stereo_camera.yaml、realsense_align_node.launch.py，副本位于 `evidence/rgbd_calibration_20261030/`。
标定包含 cam0–cam5，但彩色 `/image` 的 frame_id 是数字编号（此前样本为 `7585`），没有相应彩色 CameraInfo。因此尚不能直接确定该照片流对应哪组标定，也不能直接把 cam1 或 cam3 当作已确认的彩色相机。下一步核对原厂驱动对相机编号、图像缩放与畸变参数顺序的实际使用，再验证深度与彩色轮廓投影。
