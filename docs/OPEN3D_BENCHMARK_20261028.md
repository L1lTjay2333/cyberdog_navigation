# Open3D离线配准对比（2026-10-28）

## 结论

D盘Open3D安装可用，已对真实保存的7个关键帧进行独立帧对检查。现有自写求解器与Open3D得到非常接近的局部纠偏量，明显移动帧对仍未通过统一质量判定。没有依据将问题简单归因于自写ICP实现，也不能据此证明任何一份位姿为真值。

本轮不替换在线融合算法，不调整质量阈值，不连接机器人。已有网页和模型不被改写。下一次仍需连续的小角度转动数据；不是继续安装更多库就能证明大角度一致性。

## 对比条件

- 输入：`evidence/reconstruction/20261027_201124_805102/`的7个原始深度关键帧，各相邻帧对及1→50帧对；不是完整54帧重放，也不是轨迹闭环。
- 环境：D盘独立Python 3.11、Open3D 0.20.0、NumPy 2.4.6、SciPy 1.17.1。
- 两种方法使用相同的2 cm采样、每云最多6000点、PCA局部法向、8 cm对应距离和1 cm Huber尺度。自写实现的法向计算抽成共享函数，公式未改变。
- 每5个源点留出1个，两种求解器都不使用这些点拟合。目标云法向仍来自整份目标观测，因此不是独立传感器或独立真值验证。
- Open3D使用20次上限的鲁棒点到平面ICP；收敛条件及逐步纠偏规则与自写实现并不完全相同，不宣称两个算法完全等价。
- 用固定的初始留出有效点比较纠偏前后残差，并检查对应保留率、几何退化和纠偏幅度。不会只看Open3D输出的fitness或inlier RMSE就认定成功。
- 自写实现输出的是通过其检查后实际允许施加的变换；拒绝时返回单位矩阵，对比脚本不猜测或重造被拒绝的内部候选变换。

参考：[Open3D官方鲁棒核教程](https://www.open3d.org/docs/release/tutorial/pipelines/robust_kernels.html)，并在已安装0.20.0版本中核对接口和实际运行结果。

## 结果

下表残差为同一批留出点的点到平面RMSE，不是之前报告中的逐像素深度残差，也不是绝对测距精度。下表后三个帧对也独立执行，不代表从第1帧连续跟踪通过了所有帧。

| 参考→源帧 | 原始RMSE | Open3D候选后RMSE | 留出有效对应保留率 | 判定 |
| --- | --- | --- | --- | --- |
| 1→2 | 10.54 mm | 10.37 mm | 98.98% | 原始观测已较一致，改善不足，不值得施加微调 |
| 2→10 | 10.45 mm | 10.37 mm | 99.22% | 同上 |
| 10→20 | 11.75 mm | 11.71 mm | 99.33% | 同上 |
| 20→30 | 20.47 mm | 14.08 mm | 92.96% | 低于95%保留要求，拒绝 |
| 30→40 | 10.45 mm | 9.72 mm | 96.05% | 独立局部候选通过检查，但不是绝对精度验收 |
| 40→50 | 10.10 mm | 9.79 mm | 98.07% | 改善不足，保持原始位姿更稳妥 |
| 1→50 | 22.12 mm | 16.30 mm | 94.59% | 未通过保留率，且残差超过15 mm要求 |

关键的20→30帧对间隔约3.67秒：自写候选约2.389 cm / 0.771°，Open3D候选约2.388 cm / 0.770°。两个实现候选结果接近，但都不足以证明纠偏可靠。RMSE下降同时伴随有效对应丢失，不能只挑改善数字。

这些数据没有真实大角度转身、独立尺量或完整轨迹，无法据此判定整间房的漂移与回环能力。

## 代码、证据与验证

- `tools/benchmark_open3d_registration.py`：纯离线对比入口，无SSH、ROS或机器人控制调用；输出文件使用独占创建，避免覆盖旧结果。
- `tools/reconstruction_registration.py`：仅抽取已有PCA法向函数供对比复用，未放宽在线或实验配准阈值。
- `tests/test_reconstruction_open3d.py`：新增5项检查，覆盖非刚体矩阵、单平面退化、无重叠、已知纠偏和实际Open3D局部求解。原环境未安装Open3D时跳过该可选测试组。
- 新独立环境中56项三维相关测试通过。
- 完整结果保留本地：`evidence/reconstruction_experiments/open3d_benchmark_20261028.json`。

在项目根目录运行（输出路径须尚不存在）：

```powershell
$env:PYTHONPATH = "tools;tests"
D:\cyberdog\tools\reconstruction_runtime\.venv\Scripts\python.exe -X utf8 -m unittest discover -s tests -p "*reconstruction*.py"
D:\cyberdog\tools\reconstruction_runtime\.venv\Scripts\python.exe -X utf8 tools/benchmark_open3d_registration.py evidence/reconstruction/20261027_201124_805102 evidence/reconstruction_experiments/my_open3d_comparison.json
```

## 下一步

电脑端依赖和离线对比入口已准备好。下次开机后先恢复原厂深度与同时间位姿输入、重启8766使连续归档生效，再由人低速转动约30°并返回。先比较连续数据上的拒绝位置和几何变化，再决定是否接入Open3D或调整配准方法；不在缺乏中间画面的旧子集上反复调参追求通过。
