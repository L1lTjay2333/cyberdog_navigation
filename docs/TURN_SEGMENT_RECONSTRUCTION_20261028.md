# 转向配准与分段重建实验（2026-10-28）

## 当前交付

- 主工作台8768现有效果保留，未切换为实验配准，不改在线采集。
- 新增“先粗后细”的离线配准候选，参照[Open3D多尺度ICP](https://open3d.org/html/tutorial/t_pipelines/t_icp_registration.html)；以原位姿为初始条件。
- 新增整段配准检查及独立分段灰模对照页：http://127.0.0.1:8767/turn_segments/ 。原位姿和局部对齐版本可切换、下载。切换保留视角，另选数据段重新适合窗口。
- 分段流程只读本机原始归档，不连接机器狗、相机、ROS或SSH。

## 实际数据结果

输入：`evidence/reconstruction/20261028_163545_462434/frames.jsonl`，44帧。

SHA256：`025965fb4b3b3d832bb75de38760ce3d8fa2e84b2fa404da98dece0b793cce6f`，本轮前后不变。

43组相邻帧，以不修正的原位姿对齐通过26组，多尺度候选通过32组。这里比较的是局部几何检查通过数，不是绝对精度、完整轨迹成功率或原算法同输入求解器胜率。

连续轨迹仍仅通过前15帧。第16帧相对前帧平移约11.1厘米、旋转约3.09度；候选需约7.22厘米修正，超过保留的5厘米上限。其点面验证残差由约15.13毫米降至7.68毫米，也不足以越过修正上限。接近尾部有相邻约11.1度、10.5度转动；本记录不代表90度或360度转向验证。

因此没有发布“整段转向已经纠偏”的模型，也未把原位姿改写。数据到达延迟不等于TF与深度的同步误差，本记录无法独立确认硬件同步、外参或真实尺寸准确性。

## 分段策略

每个新帧对最近3帧局部地图配准。使用既有几何检查：重叠、可观测性、相同验证点、残差改善；局部及累计修正均保留5厘米/5度限值。优化不使用按采样索引留出的验证点，但这些验证点仍来自同一传感器，不是独立测量真值。

任何检查失败就关闭当前段，新帧从其原位姿开始下一段，不跨越该失败边继续累积。所有44帧各进入一个且仅一个段，失败帧不丢弃。各段独立锚定，不能把它们直接合并当作纠偏整屋模型。

实际分段范围：1–15、16–17、18–21、22–26、27–28、29–37、38、39、40–41、42、43、44，共12段。其中5段只有单帧，未执行跨帧修正。

32条段内边通过检查，23条实际应用非恒等局部更新，其余保留初始化位姿。灰模使用相同空间深度过滤和2厘米Tensor TSDF，未做网格平滑或贴图；各段均导出原位姿、局部对齐两个版本，面数变化不作为质量验收指标。

## 文件与复现

```powershell
cd D:\cyberdog\cyberdog_navigation\cyberdog_navigation
$env:OMP_NUM_THREADS='2'
D:\cyberdog\tools\reconstruction_runtime\.venv\Scripts\python.exe -X utf8 -B tools/reconstruction_turn_audit.py evidence/reconstruction/20261028_163545_462434 evidence/reconstruction_experiments/turn_audit_new
D:\cyberdog\tools\reconstruction_runtime\.venv\Scripts\python.exe -X utf8 -B tools/reconstruction_segments.py evidence/reconstruction/20261028_163545_462434 evidence/offline_mesh_20261028/mixed_motion_gray/turn_segments_new
```

输出目录必须不存在，不覆盖原数据。复现分段工具会复制查看器，但需要将包含Three.js的`mixed_motion_gray`目录通过本机HTTP服务打开；当前8767已经提供此服务。

实际审查报告：`evidence/reconstruction_experiments/20261028_multiscale_filtered/turn_report.json`。

实际模型：`evidence/offline_mesh_20261028/mixed_motion_gray/turn_segments/`，每段保存`frames_corrected.jsonl`（明确为派生位姿）、PLY、预览、修正矩阵及检查报告；完整来源仍为原归档。无RGB、无纹理。

新增8项测试覆盖已知误差恢复、单平面歧义、大修正拒绝、不凭空修改已对齐输入、过少点、失败断段保留所有帧、累计修正限值、位姿修改与点云坐标变换一致性。120项三维相关测试通过，浏览器逐段检查12段的两个版本显示完成且无脚本错误；截图在`evidence/reconstruction_experiments/segments_browser.png`。

## 下一阶段

分段是局部质量控制，不是终点。[Open3D的多视图重建流程](https://www.open3d.org/docs/release/tutorial/pipelines/multiway_registration.html)使用位姿图关联重叠片段。后续要检查相邻段的双向一致性及联合约束，得到可靠段间边后再做联合优化；重叠不足时保留分段。没有回到起始位置的这组数据不能拿来宣称闭环成功，实际大角度和尺寸验收仍需新的受控采集。
