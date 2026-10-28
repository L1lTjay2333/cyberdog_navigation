# 已采集数据离线处理工作台（2026-10-28）

入口：http://127.0.0.1:8768/ 。采集网页8766也增加了入口链接。

## 操作

1. 启动工作台（电脑即可，机器狗不必开机）：

```powershell
cd D:\cyberdog\cyberdog_navigation\cyberdog_navigation
D:\cyberdog\tools\reconstruction_runtime\.venv\Scripts\python.exe -X utf8 -B tools/reconstruction_workbench.py
```

2. 打开入口，选择已有完整采集记录，点击“处理所选记录”。页面显示启动、深度处理、融合、准备预览、完成或失败等实际阶段，没有编造百分比。
3. 完成后切换完整观测灰模、较高支持灰模、处理后点云。旋转、缩放、平移；切层保持视角，必要时点“适合窗口”。
4. 下载完整PLY和JSON报告。网页预览上限15万面、3万点，下载保留完整结果。大模型预览选取部分三角面，可能出现预览空隙。
5. “取消处理”只终止本工作台启动的处理进程，保留原始及部分结果。失败或取消不会发布为已完成结果。已有显示结果继续保留。

## 数据流程与文件

- 输入：`evidence/reconstruction/<采集ID>/frames.jsonl`、`summary.json`，只读。
- 独立输出：`evidence/offline_runs/<运行ID>/`，每次新建，不覆盖旧记录。
- `processed/`：空间深度异常剔除、2厘米体素去重、多帧共识点及待确认候选，衍生深度归档与报告。
- `fusion/`：原深度经同样空间过滤后，以原采集位姿进入Tensor TSDF，输出灰色表面PLY及报告。
- 表面融合不会使用严格共识遮罩硬删除新视角，避免真实新区域丢失。
- `progress.json`、`worker.log`、`exit.json`：进度、错误和进程退出证据；完成需同时满足退出码0及完成状态。
- 工作台不导入SSH、ROS或机器人接口，不会连接相机、开始采集或发送运动。
- 本机单进程任务，最长600秒，输入最多900帧、256MiB；超限明确拒绝。不是无限制整屋重建器。
- 前端依赖D盘运行环境已有Three.js，无外部CDN请求。

## 实际验证

44帧记录 `20261028_163545_462434` 完整运行成功：30,414主点云、44,161待确认候选、122,942完整观测面、53,326较高支持面。

源哈希仍为 `025965fb4b3b3d832bb75de38760ce3d8fa2e84b2fa404da98dece0b793cce6f`。

浏览器实际加载三个层、检查六个下载，未出现脚本错误；浏览器验证只发本机GET，不触发机器人或采集。测试入口：

```powershell
$env:PYTHONPATH='tools;tests'
D:\cyberdog\tools\reconstruction_runtime\.venv\Scripts\python.exe -X utf8 -m unittest discover -s tests -p '*reconstruction*.py'
node tests/offline_workbench_browser.cjs
```

浏览器检查依赖上述44帧已完成结果及8768服务，验证截图位于 `evidence/offline_runs/workbench_browser.png`。

## 限制与后续

这一步交付可操作的数据处理流程，不等于精度问题已解决。未修正位姿、未做闭环、未做灰模平滑、无材质与纹理。融合支持权重不代表独立观测次数或真实精度。单帧候选可能为真实新表面，仍保留供审查。后续优先验证转向时配准、时间对齐与真实尺寸，再扩大扫描范围。
