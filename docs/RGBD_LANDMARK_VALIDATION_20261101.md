# 照片与深度实体对应点验证

新增tools/rgbd_correspondence_fit.py，为离线诊断工具；不连接机器人、不修改标定、不自动导出已验证rgbd_calibration.json。

输入JSON字段：depth_xyz_m（Nx3米制深度坐标）、color_uv（Nx2彩色像素）、groups（每个对应点所属摆放组）、validation_groups（整组留出）、measurement_evidence（真实标注来源）、color_intrinsics、distortion、initial_depth_to_color。

必须至少24个对应点、3组摆放，拟合至少16点，留出至少8点。拟合三维点须空间分布充分；单一平面拒绝。固定内参和畸变，仅允许小幅刚体外参残差修正。稳健优化的参数界限为各旋转分量5度、各平移分量5厘米。

输出原标定与候选标定在拟合组及独立检查组的像素误差。独立组95分位不超过3像素且中位数不变差时才通过初步筛查。该筛查不是最终真机标定认证，spatial_registration_verified始终为false。

运行：python tools/rgbd_correspondence_fit.py landmarks.json new_report.json。输出必须是新文件。

3项合成测试通过：已知小幅外参回收、留出组错误拒绝、单平面及无独立组拒绝。尚未输入本项目真实实体角点，也未得到真机候选修正。

现场需一块表面哑光、有清晰角点/纹理的硬纸板或纸盒。目标要在彩色图像里完整可见，并有背景深度间隔。由于RGB视野上仰，先查看照片确认取景，不能依据狗头朝向猜测目标完整入镜。分多次保持机器狗静止，改变目标摆放位置/倾角与距离，获得不同组；逐组取景确认后采集。不要用深度缺失边界自动代替实体角点。

输入标注与深度重投影提取仍需核验；本次新增的是数值求解与独立检查工具，不代表完整自动标定已经完成。
