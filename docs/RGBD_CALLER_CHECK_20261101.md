# 原厂实际对齐调用检查

本轮只读访问机器人，没有切换相机、采集或发送运动。

标定文件/params/camera/calibration/params_extrinsic.yaml的SHA256为48dec09592c8c848e99a86a165d73245854697f1e098151df1fe59e2b484b9a2，与电脑归档相同，因此不是使用了另一份标定文件导致的差异。

实际程序/opt/ros2/cyberdog/lib/realsense2_camera/realsense_align_node保留readAlignParameters与publishAlignedFrame符号。已保存反汇编：evidence/rgbd_calibration_20261030/native_caller/parameters_and_call.txt。

- 0x7f744：fmov s0,#1.0，传入深度缩放1.0。
- 0x7f748/0x7f74c：目标内参位于对象+0x44，外参位于+0x74。
- 0xa71f0/0xa71f4：将4写到对象+92，即rs2_intrinsics的model字段。
- 存档rs_types_v250.h定义Brown-Conrady为4，Kannala-Brandt4为5。此前文档把Brown写为5，已纠正。
- 参数装载可见cv::invert，以及逆矩阵元素转为float写入外参区；详细分支与原厂实际输出尚未独立核实，不能把数学矩阵方向直接判为已验收。

内核PTX目标畸变分支只处理模型1与3。因此模型4仍没有应用Brown畸变。这是已检查软件路径的行为，不等同于相机真实成像模型应当无畸变。原先native_ptx_public_call_assumption分支仍只是诊断，其针孔计算数值无需因枚举更正而改变。

空间对齐未通过；不生成spatial_registration_verified=true文件，也不启用真实照片辅助几何结果。下一步应对实际对齐输出与同批照片做交叉核验，并检查彩色流裁切/缩放及标定系数排列，不继续通过目测平移标定凑合轮廓。

## 现有输出检查

20261101_155346_696086_inventory.json记录7秒只读订阅：原始深度81帧、彩色158帧。aligned_depth_to_rgb/image_raw及aligned_depth_to_infra1/image_raw均有发布者但未收到帧；camera_align生命周期为unconfigured [1]。因此现有输出不能用于原厂对齐结果交叉验证。没有执行生命周期切换。

使用新静止7组数据比较原系数顺序与radtan候选重排，重排后仍存在目标上缘偏移；不能把问题仅归因于系数排列，也不能凭目测把候选设为已验证。

诊断页：http://127.0.0.1:8772/，文件位于evidence/reconstruction/20261101_154345_752959/alignment_diagnostic/index.html，内嵌照片避免图片路径丢失。所有候选分支保留并明确标记未验收。
