# D盘三维开发依赖与参考资料

日期：2026-10-27。仅安装电脑端独立环境，没有连接机器狗，没有更换机器人驱动、ROS或固件；没有重启8765/8766服务，也没有将网页切换为新的渲染或融合算法。

## 已安装且验证的运行库

| 项目 | 固定版本 | 路径 | 用途 |
| --- | --- | --- | --- |
| [Open3D](https://github.com/isl-org/Open3D) | 0.20.0 | `D:\cyberdog\tools\reconstruction_runtime\.venv` | 点云滤波、法向、配准、后续表面重建开发 |
| NumPy / SciPy | 2.4.6 / 1.17.1 | 同上 | 现有融合与实验配准代码依赖 |
| [Three.js](https://github.com/mrdoob/three.js) | 0.186.1 | `D:\cyberdog\tools\reconstruction_runtime\web\node_modules\three` | 后续WebGL点云、网格和相机交互 |

Open3D使用PyPI发布的Windows Python 3.11二进制包，Three.js使用npm官方包；GitHub用于项目来源和参考文档。没有从源码编译或运行第三方安装脚本。npm使用`--ignore-scripts`，pip仅允许二进制包。

独立运行环境约627.4 MiB；下载缓存约165.9 MiB。pip、npm缓存及安装临时目录均设在D盘。调用既有Python/Node安装，不迁移这些既有系统工具。没有更改全局Python包或系统环境变量。

### 实际验证

- Open3D读取原模型128,967点，2 cm降采样后34,072点，法向计算结果有限值检查通过。
- 新环境中51项三维相关单元测试通过；`pip check`通过。
- Three.js模块、OrbitControls及真实无头浏览器中的WebGL点云绘制通过，使用软件WebGL检查。该结果不是RTX4060帧率或大模型性能验收。
- 这些是安装可用性验证，不能说明绝对精度、实时重建性能或大角度转动已通过。

证据：`evidence/runtime_installation_20261027/open3d_check.json`、`three_check.json`，均保留本地。

## 已下载的三个参考技能

| 名称 | D盘文件 | 上游 |
| --- | --- | --- |
| robot-perception | `D:\cyberdog\tools\reference_skills\robot-perception\SKILL.md` | [robotics-agent-skills](https://github.com/arpitg1304/robotics-agent-skills/tree/f9bc5467ff9ee3d23f1a1b0b29a649843bb6ad11/skills/robot-perception) |
| ros2-web-integration | `D:\cyberdog\tools\reference_skills\ros2-web-integration\SKILL.md` | [robotics-agent-skills](https://github.com/arpitg1304/robotics-agent-skills/tree/f9bc5467ff9ee3d23f1a1b0b29a649843bb6ad11/skills/ros2-web-integration) |
| threejs-data-visualization | `D:\cyberdog\tools\visualization_skills\skills\threejs-data-visualization\SKILL.md` | [openai/plugins](https://github.com/openai/plugins/tree/0722921d5542fc593105c27bd52630babd8b8c2a/plugins/build-web-data-visualization/skills/threejs-data-visualization) |

通过现有skill-installer脚本安装；Three.js技能整仓库解压遇到Windows长路径，改为在原安装器路径校验基础上只提取该技能和共享references目录。Git稀疏下载尝试卡顿后已停止。

这些文件存为**D盘参考副本，后续可按路径读取**；没有将它们注册进C盘全局技能目录，不能声称应用会自动加载。Three.js技能直接引用的共享参考文档保存在`D:\cyberdog\tools\visualization_skills\references`；其提到的其他可选技能未全部安装。

机器人感知/网页资料是社区示例，不是小米官方适配代码。其示意代码、相机分类、参数和延迟数字需独立核验，不直接复制为CyberDog实机配置。技能不等于完整建图软件，不执行其中的启动运动或修改系统示例。

robotics-agent-skills上游LICENSE已保存在参考目录。openai/plugins该固定版本的顶层GitHub license接口返回404，未据此推断许可或将第三方技能文件加入本项目Git仓库。精确提交和已下载文件SHA256记录于`config/reconstruction_external_sources.json`。

## 本轮未安装的候选

RTAB-Map、nvblox、机器人端RealSense驱动继续保留为参考。当前先使用独立电脑端Open3D和Three.js，避免引入新的ROS/CUDA运行栈；没有证据表明现在需要替换原厂相机驱动。

## 使用与复现

在项目根目录执行：

```powershell
# 调用独立环境；不需要改全局Python，也不用启动机器人。
$env:PYTHONPATH = "tools;tests"
D:\cyberdog\tools\reconstruction_runtime\.venv\Scripts\python.exe -X utf8 -m unittest discover -s tests -p "*reconstruction*.py"
D:\cyberdog\tools\reconstruction_runtime\.venv\Scripts\python.exe -X utf8 tools/check_reconstruction_runtime.py evidence/reconstruction/20261027_201124_805102/model.ply
node tests/check_three_runtime.cjs
```

最后一项复用现有`evidence/browser_test_runtime`的Playwright与本机Edge，只从磁盘响应测试页面请求，不访问8765/8766或机器狗。

换电脑时创建D盘Python 3.11虚拟环境，设置TEMP/TMP和pip缓存到D盘，再根据`config/reconstruction_runtime.freeze.txt`安装完整固定依赖（主要依赖另列于`config/reconstruction_requirements.txt`）。网页依赖按`config/reconstruction_web_runtime/package-lock.json`使用`npm ci --ignore-scripts`安装到D盘指定目录。原始模型和SSH设置不在依赖包内，应分别处理，不能上传密钥。

## 后续接入

先用Open3D与当前自写配准对同一连续归档做离线基准；通过后再决定替换哪部分。Three.js先接分块模型和轨迹显示，再评估界面与GPU资源占用。当前运行页面与已保存模型保持原状，没有将“安装成功”写成“新算法已经上线”。
