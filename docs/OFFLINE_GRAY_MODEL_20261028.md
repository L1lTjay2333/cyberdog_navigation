# 离线灰色表面模型（2026-10-28）

## 已得到的结果

使用会话`20261028_163545_462434`中完整保存的44帧深度图、内参和原时间戳相机位姿，生成无贴图、无外部材质的灰色几何网格。没有连接机器人或启动新采集，没有覆盖原始数据或现有网页模型。

完整输出在`evidence/offline_mesh_20261028/mixed_motion_gray/`：

| 文件 | 内容 |
| --- | --- |
| model_gray.ply | 灰色表面，746,012顶点、1,429,733三角面，约56.6 MB |
| model_gray.obj | 同一几何表面，无MTL、无贴图，约111.8 MB |
| preview_gray.ply | 3 cm顶点聚类后的轻量预览，28,760顶点、96,031三角面 |
| view_gray.html | Three.js离线浏览页面，拖动旋转、缩放和平移 |
| gray_preview.png | 实际浏览器截图 |
| mesh_report.json | 输入来源、参数、数量和限制 |

页面与模块均从本机文件提供，不使用CDN或机器人接口；用HTTP从该输出目录启动本地服务后打开view_gray.html。预览经过聚类，不能用于尺寸验收；完整模型另有PLY/OBJ。

## 数据怎样变成模型

1. 读取顺序归档，核对时间递增、坐标系一致、压缩数据边界和原始内参。
2. 深度统一成米，剔除非有限值及0.15–4米之外的值，不设置经验距离补偿。
3. 每4个像素取样，依据相机内参反投影成相机坐标点。
4. 在深度图网格中将相邻有效点连成三角面；深度跨度超过4 cm或任意边长超过10 cm的三角面被排除，避免跨过门边/杂物边缘强连面。该规则是实验固定参数，不代表所有物体都已正确分割。
5. 使用原四元数和位移，将相机局部表面放到vodom坐标。
6. 合并网格、去掉完全重复和退化的元素，计算法向，输出统一灰色PLY与纯几何OBJ。

这是**多帧深度表面叠放原型**，不是完成跨帧融合的TSDF模型：不同帧的重叠表面仍可保留，原位姿误差也会形成粗糙和重影。未观察区域保持缺失，不把孔洞补成虚构墙面。没有RGB，不能称作彩色房间扫描。

## TSDF尝试与当前限制

曾按Open3D官方RGB-D integration接口尝试深度TSDF。在当前电脑环境中，已知1米平面的最小合成测试也返回空表面。Open3D 0.20.0、独立对照0.19.0，以及NumPy 1.26.4对照均复现；未确证底层原因，不能认定为某个版本已知Bug，也不能将其归咎于真实机器人输入。

对照依赖只放在D盘`tools/reconstruction_runtime/compat_open3d019`，未替换正在运行的正式环境。当前交付明确采用深度网格三角化，**不宣称TSDF已跑通**。后续需要解决跨帧配准、表面去重/融合，再考虑TSDF或其他表面融合方式。

参考算法文档：https://www.open3d.org/docs/release/tutorial/pipelines/rgbd_integration.html 。文档解释路线，不等于本机TSDF已验收。

## 验证

- 新增5项灰模测试：米制深度、相机/世界变换方向、完整四元数、压缩输入拒绝、已知平面出面及坐标系变更拒绝（其中部分同一测试覆盖多个条件）。
- D盘独立环境83项三维相关测试通过。
- 实际生成并写出PLY/OBJ，浏览器加载轻量灰模无脚本错误；截图已目视检查。粗糙、缺口与分层可见，没有将其称为精确房间模型。
- 本次实际建模及文件写出约37.6秒，不代表实时建模帧率；OBJ文本输出占用较大。

## 可复现入口

```powershell
$env:PYTHONPATH = "tools;tests"
D:\cyberdog\tools\reconstruction_runtime\.venv\Scripts\python.exe -X utf8 -m unittest discover -s tests -p "*reconstruction*.py"
D:\cyberdog\tools\reconstruction_runtime\.venv\Scripts\python.exe -X utf8 tools/reconstruction_mesh.py evidence/reconstruction/20261028_163545_462434 evidence/offline_mesh_20261028/new_gray_model
```

输出目录必须不存在。程序生成完整PLY、OBJ和报告；轻量预览及HTML是本次另外生成的查看产物，当前CLI尚不自动生成它们。数据和模型留本地，不进Git，交接时另行复制。

## 下一步软件工作

先在同一份44帧数据上检查错位和深度边界，完成跨帧几何融合，再减少重复表面和大OBJ体积；不靠漂亮材质遮住误差。之后将灰模查看入口接入8766页面，最后才扩展到房间级扫描。此次不需要新的实机操作。
