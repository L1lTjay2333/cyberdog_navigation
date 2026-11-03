# 彩色与深度标定源码核对

## 已确认的源码行为

本地原厂相机源码：
`D:/cyberdog/official_stair_sources/cyberdog_ros2/cyberdog_interaction/cyberdog_camera/cyberdog_camera/src/camera_service/rgb_stream_consumer.cpp`。
`publishImage()` 将帧计数转为字符串写入 `header.frame_id`，并发布 bgr8。因此实采 `frame_id=7585` 不能解释为镜头或 TF 坐标系名称。
同目录 `main_camera_node.cpp` 调用 `openCamera(0)`；Argus 设备编号与标定文件 cam 编号不能仅凭数值相等建立对应。

已获取小米官方 `MiRoboticsLab/realsense-ros` 源码固定版本：
`cc68a90fc888041fc2a08aafc3b401fb88dbd6d8`。
只下载到电脑 D 盘，没有安装到机器人。

`realsense2_camera/src/realsense_align_node/parameters.cpp`：
- readAlignParameters 从 params_extrinsic.yaml 加载标定。
- 第184–187行明确读取 cam3 的 T_cn_c0、intrinsics、resolution、distortion_coeffs。
- 对 T_cn_c0 求逆后填充 _z_to_other；旋转元素的内存布局也必须按调用方的约定解释。
- 畸变数组原顺序传入 Brown-Conrady，不应凭常见文件格式自行重排。

`align_realsense_node.cpp` 的 publishAlignedFrame 使用当前深度视频流内参和 _z_to_other、_other_intrinsics 调用 align_depth_to_other。
以上证明官方对齐路径选择 cam3，不单独证明当前 /image 流、已安装二进制版本、裁切/缩放方式和该标定一致。

源码副本、GitHub目录树及真机标定副本：`evidence/rgbd_calibration_20261030/`。
官方路径：
https://github.com/MiRoboticsLab/realsense-ros/blob/cc68a90fc888041fc2a08aafc3b401fb88dbd6d8/realsense2_camera/src/realsense_align_node/parameters.cpp

## 下一项验收

用已保存的静止配对帧和实际深度 CameraInfo 做候选投影，检查椅背、墙角等共同轮廓是否一致；同时核实原厂已安装对齐程序的标定使用方式。没有通过前，不能将候选彩色点云或灰模标记为对齐完成。
不通过时先检查坐标方向、相机对应、畸变和缩放；不能通过目测调外参掩盖错误。仅时间匹配不足以构成RGB-D配准通过。

## 离线投影检查结果

新增 `tools/rgbd_projection_check.py`，使用真实深度内参回投影深度不连续边界，再采用 cam3 内参及 Brown-Conrady 畸变投到彩色图像。输出的是诊断叠加图，不是完成配准的彩色模型。
运行：`python tools/rgbd_projection_check.py evidence/rgbd_samples/20261030_171129_366554`。
4 对图像的结果位于对应记录下 `projection_check/`。绿色点为投影的深度边界，原照片为背景。
分别保留官方源码数组按列主序解释、普通数学逆矩阵、直接矩阵三种诊断结果，没有按视觉观感自动挑选或修改标定。
首尾两对官方数组解释预览均存在可见轮廓错位，空间配准验收未通过。部分绿色边界来自无效深度边界或遮挡，不等于实体轮廓；不能据所有绿点与照片边缘的距离反推标定精度。
投影函数已用独立简单输入检查主点投影、米制平移、相机后方剔除，原厂旋转矩阵正交性也通过数值检查；这些检查不代表实采空间配准通过。

真机只读 strings 确认 `/opt/ros2/cyberdog/lib/realsense2_camera/realsense_align_node` 含 `cam3`、`T_cn_c0`、`/params_extrinsic.yaml` 和 readAlignParameters 符号，与已查源码的标定选择吻合。字符串不能证明所有计算与当前官方版本完全一致。

## 彩色时间戳的限制

本地原厂 rgb_stream_consumer.cpp 的 processBuffer 中，在处理图像前调用 CLOCK_REALTIME，将当时系统时间写入图像 header。该源码行为不是直接记录传感器曝光时刻；尚需核实已装二进制是否完全相同。
因此现有 4–23 毫秒只应表述为消息时间戳差，而非曝光同步误差。静止数据可继续用于空间校验；后续移动采集应先采用停稳再采的验证步骤，并单独确认动态时延。
后续优先核实当前对齐内核的矩阵布局和实际彩色流的裁切/缩放路径，必要时以原厂对齐节点输出交叉检查；本次没有切换该节点或修改标定。

## 前景轮廓检查修正

原诊断把无效深度邻域边界与深度跳变混合，而且固定选取右侧/下侧像素，可能将背景端点作为物体轮廓。已改为只在两个深度均有效且相差超过 0.06 米时选取较近端点；无效邻域边界单独输出橙色诊断图。未修改原始采集数据、标定或重建结果。
新结果保存在 `projection_foreground_check/`，旧 `projection_check/` 保留。
四帧原边界数量分别5577、5482、5565、5615；新前景边界数量分别689、575、628、632。这是诊断口径变更，不是建模精度提升比例。
首帧新预览中椅子部位可靠前景跳变边界很少。因此此前“存在错位”的视觉判断不能单独据此推导出外参错误；当前结论应为：空间对齐尚未验证，现有椅子观测的可靠轮廓不足。深度缺失边界并非一律噪声，只是不能单独作为已知物体轮廓。
4项测试通过：薄前景物体两侧近端选择、孔洞独立标识、翻转/转置方向不变性、已知米制平移投影及相机后方剔除。
下一轮宜在相机可见高度放置有明显边缘的哑光纸板/纸盒，与背景留出深度差，静止采一组照片和深度以验证投影；本次不需要为了诊断算法重复绕椅子扫描。跨相机遮挡仍未处理，不能自动选择三种外参解释中观感最佳者作为真实标定。

## 离线可见性及单位复核

读取 Intel RealSense SDK v2.50.0 原始接口，存档为 `rs_sensor_v250.h`、`rs_types_v250.h`、`rsutil_v250.h`。rs2_extrinsics 的 rotation 明确是列主序，translation 明确为米，Brown-Conrady 系数顺序为[k1,k2,p1,p2,k3]。
来源：https://github.com/IntelRealSense/librealsense/blob/v2.50.0/include/librealsense2/h/rs_sensor.h

小米公开调用处使用 uint16 深度数据和 z_scale=1.f，进入 align_cuda_helper。当前存档只有该内核声明，没有实现；内核是否内部换算单位、是否使用SDK相同旋转布局尚未证明。故 `source_array_column_major` 仅是将源码数组按SDK约定解释的假设，不能称原厂真机等效投影，也不能据此认定原厂存在单位错误。

新增目标像素深度缓冲诊断：全有效深度转换到目标相机，按投影后四舍五入像素取最近Z；候选边界若比同像素最近表面远超过0.02米，不再画入叠加图。使用目标相机Z而非原深度相机Z。该方法不补洞、不扩张像素，尚不能完整模拟跨相机可见性；2厘米是诊断容差，并非精度指标。
重跑5组共15对数据。SDK列主序假设分支累计剔除11674个遮挡候选边界样本（含多帧重复观测），不能解释为删除11674个真实噪点或建模精度提升比例。原始采集数据未改动。
6项测试通过，新增了同像素远处表面剔除、旋转后目标相机Z排序检查。离线对照页已更新，明确标记原厂内核待核对。空间配准状态仍为未验证。
