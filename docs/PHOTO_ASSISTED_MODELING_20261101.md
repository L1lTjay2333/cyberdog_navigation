# 照片辅助局部几何建模

入口：离线工作台处理方式“照片辅助局部灰模（需已验证标定）”。命令入口：

`python tools/reconstruction_offline_worker.py <session> <new_output> --mode color --first-frame 1`

new_output须预先创建为空目录。正式工作台会自动创建它。

## 数据链路

1. 正式采集color_sample保存照片与深度的时间关联；没有对应照片的旧记录不能用于本模式。
2. 使用米制depth_to_color外参、彩色Brown-Conrady内参投影，最近深度遮挡检查后赋予深度点颜色。没有填补缺测区域。
3. 选择起始原始帧之后最多10个彩色关键帧，至少3个；局部范围不能代表整段扫描。
4. Open3D Colored ICP在4/2/1厘米多尺度同时利用颜色和几何。每五个源点留一个不参与优化，之后由现有几何检查评估重叠、可观测性、修正幅度和残差。失败终止，不强制拼接。
5. 将通过的位姿修正应用于深度关键帧，用TSDF生成原位姿与颜色辅助位姿两个灰模。最终无需贴材质。输出frames_color_corrected.jsonl、color_alignment.json和fusion/aligned_gray.ply。

## 标定文件

每次会话内需要rgbd_calibration.json，字段如下：

- spatial_registration_verified：只有独立实测对齐验证通过后才可为true。
- verification_evidence：实测验证记录路径或说明，不得为假想验证。
- translation_unit：meter。
- model：brown_conrady。
- depth_to_color：4×4刚体矩阵，深度相机点到彩色相机，平移单位米。
- depth_frame：与记录source_frame一致。
- depth_intrinsics：[fx,fy,cx,cy]，与记录一致。
- color_intrinsics：[fx,fy,cx,cy]。
- distortion：[k1,k2,p1,p2,k3]。
- color_size：[width,height]。

这是显式提供的验证证据，不是软件自行证明标定正确。当前真机尚没有通过验证的该文件，禁止把既有不同投影假设任选一个填入并设为已验证。

## 验证和实际限制

13项照片解码/几何投影/时间拒绝/标定拒绝/工作台/采样测试通过。另用有纹理的三个互相垂直平面合成数据执行真实Colored ICP：已知平移[4,-3,2]毫米，估计约[3.998,-3.003,2.000]毫米，留出点几何检查通过。该结果是合成测试，不是真机精度。

当前照片消息时间戳并非已证实曝光时间，50毫秒窗口仍可能有运动错配。真实空间对齐、曝光时序、多角度彩色采集仍待验收。照片对无纹理、反光、缺失遮挡表面不保证改善，TSDF分辨率仍为2厘米；本实现不是摄影测量超分辨率、全局闭环或缺失表面生成。

新worker由工作台启动时会读取新代码；已运行的旧工作台进程需要重启以接受color模式。页面文件刷新可显示新选项，但不等于后端已重启。

官方参考：https://www.open3d.org/docs/release/tutorial/pipelines/colored_pointcloud_registration.html
