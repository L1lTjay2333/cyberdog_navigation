# 331 帧近景建模与位姿检查（2026-10-29）

## 本轮结论

此前只将重点放在边缘噪声和移动位姿，忽略了明显的深度距离相关不稳定。
同一静止窗口第 1–10 帧，9 对相邻帧双向重投影共 18 次检查，分距离统计如下。
数值为“各帧对绝对深度差中位数的中位数”，不是实测绝对精度；仍含遮挡、采样和位姿误差。

| 相机光轴深度范围 | 重投影差异 |
| --- | ---: |
| 0.15–0.5 m | 1.03 mm |
| 0.5–1 m | 2.31 mm |
| 1–2 m | 10.31 mm |
| 2–4 m | 70.28 mm |

因此新增每帧相机坐标下的 2 m 深度限制对照。不是以起点画一个 2 m 圆，机器人移动时可逐步覆盖新的区域。
完整原始归档不变；远处数据暂不进入该版本 TSDF，但仍在原始记录内。
这也减少了覆盖范围，不能将面数下降当作精度提升。2 m 是本次数据的工作范围试验，不是传感器通用规格或已完成标定。

## 产物与验证

- 原档：`evidence/reconstruction/20261029_162923_107730/frames.jsonl`
- 331 帧 SHA256：`69b36327d302279f085000089a91c612b020d53e26f745433c64b0c7e8293cc1`，处理后核对未变。
- 原范围审查：`evidence/reconstruction_experiments/20261029_pose_audit_full/turn_report.json`
- 近景审查：`evidence/reconstruction_experiments/20261029_pose_audit_near/turn_report.json`
- 近景产物：`evidence/reconstruction_experiments/20261029_noise_audit_disjoint/near_range/`
- `near.ply`：205,143 面；`supported.ply`：112,308 面。
- 原范围基线：同一原档的 `evidence/offline_runs/20261029_163200_187398/`，1,105,910 面。
- 三层对照及下载：http://127.0.0.1:8769/near_range/ 。8766、8768 已增加明确标注日期的入口。
- 浏览器实测：三层切换、三个下载可读取、无页面错误；同一视角截图已保存为 `near.png / baseline.png / supported.png`。
- 图像复核：远处大范围毛刺层显著减少，近景仍有粗糙表面；座面缺失和整圈对齐尚未解决。
- 137 项三维相关测试通过，新增范围排除、空结果不造面、非法范围拒绝检查。

## 位姿结果：尚未采用整圈修正

两次审查使用相同 ICP / 检查阈值，只改变输入深度范围。输入点集不同，因此通过数量不能直接当成精度改善比例。

| 检查 | 原范围 4 m | 近景 2 m |
| --- | ---: | ---: |
| 原位姿相邻对通过数 / 330 | 3 | 182 |
| 局部多尺度候选通过数 / 330 | 14 | 238 |
| 从首帧连续通过的帧数 | 1 | 11 |

近景第 12 帧连续对齐中，候选残差由约 12.21 mm 降到 7.08 mm，但保留验证对应点比例只有 93.33%，低于现有 95% 要求，因此未应用。
本轮模型沿用原位姿。没有把 238 个局部成功误报为全轨迹成功，也没有合并断开的修正段。

参考 Open3D 官方[多视角配准](https://open3d.org/docs/latest/tutorial/pipelines/multiway_registration.html)：分清相邻约束与不确定闭环边，再做联合优化。
下一步应针对近景建立可靠的分段与重叠约束，处理第 12 帧的对应点丢失及转弯段，再验证重建；不应将失败边强制设为可靠边。

## 复现

在仓库根目录 PowerShell，使用已安装的独立 Python 环境；输出目录必须是新目录。

```powershell
$env:OMP_NUM_THREADS='2'
$env:PYTHONPATH='tools'
$py='D:/cyberdog/tools/reconstruction_runtime/.venv/Scripts/python.exe'
& $py -X utf8 tools/reconstruction_turn_audit.py evidence/reconstruction/20261029_162923_107730 evidence/reconstruction_experiments/pose_near_new --max-depth 2
& $py -X utf8 tools/reconstruction_range_compare.py evidence/reconstruction/20261029_162923_107730 evidence/offline_runs/20261029_163200_187398 evidence/reconstruction_experiments/range_new
```

查看已生成结果时，在独立终端启动静态服务（若 8769 已在运行，无需重复）：

```powershell
& 'D:/cyberdog/tools/reconstruction_runtime/.venv/Scripts/python.exe' -m http.server 8769 --bind 127.0.0.1 --directory evidence/reconstruction_experiments/20261029_noise_audit_disjoint
```

## 修改边界

`fuse_depth_records` 增加可选 `max_depth`，默认保持 4 m；现有工作台默认任务和在线采集未改范围。
本轮所有工作在电脑离线完成，无机器人连接、运动、原厂库或参数修改，无 GitHub 推送。
不生成未观测到的椅面，不将源文件删除，不承诺仅延长扫描就能补齐遮挡区域。
