# 毛刺来源离线对照

输入331帧椅子绕行归档，原始SHA256保持`69b36327d302279f085000089a91c612b020d53e26f745433c64b0c7e8293cc1`。本轮不访问机器人。

网页：http://127.0.0.1:8769/ ，可切换静止/移动及现有处理/排除突变边缘，共4种结果；下载完整PLY。主工作台不替换为实验版本。

## 选段与结果

自动选取不重叠、相同10帧规模的两段。静止段1–10帧，最大位移0.91厘米、角度0.46度；移动段11–20帧，最大位移54.52厘米、角度2.07度。位移均来自里程计，没有独立真值。

| 指标 | 静止10帧 | 移动10帧 |
|---|---:|---:|
| 首尾重投影绝对深度差中位数 | 8.93毫米 | 25.47毫米 |
| 现有融合连通部分数量 | 2,165 | 2,526 |
| 排除边缘后连通部分数量 | 1,790 | 1,975 |
| 现有小连通部分面积占比 | 0.836% | 0.944% |
| 排除边缘后小连通部分面积占比 | 0.711% | 0.736% |

小连通部分指面积小于0.0004平方米，可能是真实细节，不能直接称为噪声。重投影指标包含遮挡、视角与采样差异，不能当作绝对尺寸精度；两个窗口视野不同，并非只改变运动的严格因果实验。

结论边界：静止片段本身存在碎片，移动后首尾一致性变差，时间长度不足不是目前的首要解释。不能给出“多少百分比毛刺由算法造成”的结论，也未解决座面缺失。

## 边缘排除实验

在现有空间过滤后，以4邻域无效或深度差超过`0.05+0.01*z`米标记边界，分成内部/边缘两层，严格覆盖原有效像素，不生成新深度。该阈值仅作诊断，不是相机标定结果。

内部层单独融合时碎片减少，也可能损失真实扶手、金属框架和物体边缘，故未默认启用，不把网格更少当作更准确。没有网格平滑、补洞或位姿纠正。

输出：`evidence/reconstruction_experiments/20261029_noise_audit_disjoint/`。早期试运行`20261029_noise_audit`选段有重叠，不作为最终对照；保留供追踪。

## 复现与验证

```powershell
cd D:\cyberdog\cyberdog_navigation\cyberdog_navigation
$env:OMP_NUM_THREADS='2'
D:\cyberdog\tools\reconstruction_runtime\.venv\Scripts\python.exe -X utf8 -B tools/reconstruction_noise_audit.py evidence/reconstruction/20261029_162923_107730 evidence/reconstruction_experiments/noise_audit_new
D:\cyberdog\tools\reconstruction_runtime\.venv\Scripts\python.exe -m http.server 8769 --bind 127.0.0.1 --directory evidence/reconstruction_experiments/noise_audit_new
```

输出目录必须不存在。复制D盘已有Three.js依赖，只供本机查看，安装环境无需新增依赖。新增3项层分割/选段测试；127项三维相关测试和四种浏览器显示/下载检查通过。

下一步以不可靠边缘标记/融合权重为实验方向，同时保留真实细框架证据；移动位姿对齐仍单独验证。暂不要求用户重新扫描。
