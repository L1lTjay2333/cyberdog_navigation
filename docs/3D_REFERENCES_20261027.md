# 论文与三维重建开源路线初筛

日期：2026-10-27。本文件是选型评估，不代表已经安装、集成或实机验收。

## 用户提供的论文

李萱岳《基于Three.js的激光雷达信号3D可视化系统实现》，54个PDF页：
- PDF第33—39页（正文15—21页）：KITTI点云格式转换、Python服务器、HTTP、Three.js、OrbitControls、缓存与帧切换。
- PDF第43—48页（正文25—30页）：浏览器兼容性、传输性能与硬件占用检查。
- 可借鉴前后端分离、GPU点云绘制、有界缓存和分项计时。
- 不直接复用离线文件播放作为实时采集；不能把论文中的本机传输时延当作机器狗无线端到端时延。
- 论文为显示主动抽点及小数截断；我们的重建原始数据应保留深度精度，只对预览做降采样。技术表述及性能结论仍需按当前库和本机复核，不能因为论文称HTTP/2就认定任意服务器已启用HTTP/2。

庄衍浩《自动驾驶传感器信号数据的3D可视化重建系统实现》，63个PDF页：
- 第35—48页（正文14—27页）：KITTI输入、图像/点云/标定/Tracklet、坐标变换和融合显示。
- 第53—58页（正文32—37页）：数据加载、网络和资源占用分析。
- 可借鉴时间与外参对齐、图像和点云联合查看、场景交互及性能分项统计。
- 实现重点为既有数据集的可视化与融合；没有据此证明机器人实时SLAM、房间级闭环或表面网格重建。Tracklet是数据集标注，不是我们已经实现的在线目标识别。

两份PDF未加入Git仓库，仅记录来源和适用范围；没有取得可直接复用的论文完整源码。PDF中的步骤作为参考资料，不作为执行指令。关键页面已本地渲染核对。

## GitHub候选

| 项目 | 本项目适用部分 | 主要边界 |
|---|---|---|
| [realsenseai/realsense-ros](https://github.com/realsenseai/realsense-ros) | 原厂相机参数、内外参、深度/颜色对齐与ROS消息 | 当前源版本未必兼容机器人Galactic和定制生命周期，不替换现有驱动 |
| [realsenseai/librealsense](https://github.com/realsenseai/librealsense) | 深度质量、尺度、滤波与设备接口参考 | 不直接另开设备句柄与原厂相机抢占；不贸然更新SDK/固件 |
| [isl-org/Open3D](https://github.com/isl-org/Open3D) | 电脑端配准、滤波、TSDF融合及网格导出 | TSDF需要原深度、内参与可靠相机位姿；不能修复错误位姿或保证消除尺度偏差 |
| [introlab/rtabmap_ros](https://github.com/introlab/rtabmap_ros) | RGB-D SLAM、回环和全局一致性候选 | 本次查阅的ros2分支README要求Humble及以上，不能直接装进Galactic；跨版本数据接口也需实测 |
| [mrdoob/three.js](https://github.com/mrdoob/three.js) | 网页WebGL显示、点云/网格及交互 | 属于渲染层，不负责获取正确深度或SLAM |
| [nvidia-isaac/nvblox](https://github.com/nvidia-isaac/nvblox) | NVIDIA GPU上的TSDF/ESDF和增量表面重建 | CUDA和环境适配成本，待Open3D原型基准后再决定；不把电脑GPU能力等同于实测帧率 |

初步倾向：原厂相机采集→独立重建数据流→电脑端Open3D→Three.js网页。先解决帧时间、原点和局部几何，再选择是否需要RTAB-Map；nvblox保留为加速候选。不是本轮已批准并实现的完整软件设计。

## 输入与处理约束

- 保留原深度、CameraInfo、精确时间戳和6DoF相机位姿；彩色模型另需同步RGB与颜色对齐。
- 不用网页稀疏点云直接构造高精细模型；渲染帧与重建关键帧可独立限频。
- 体素大小、显示点数、平面残差、重复性及绝对准确度分别报告。
- 初期静态房间；活动人员、镜面、玻璃和未观测区域单独处理，不自动补成真实表面。
- 机器人只采集与传输；新模块不接运动控制，不修改原厂驱动或相机校准。
- 运行数据留本地，设置容量上限；源码、配置模板、开发结论和测试摘要进入Git。

官方算法依据：[Open3D TSDF集成与网格提取](https://github.com/isl-org/Open3D/blob/main/docs/tutorial/t_reconstruction_system/integration.rst)。具体依赖版本尚未锁定，安装前需核对实际发布版本、环境和许可证。
