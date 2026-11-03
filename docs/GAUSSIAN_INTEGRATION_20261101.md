# 高斯泼溅接入第一阶段

## 路线与当前状态

先从真实多角度彩色照片估计相机位姿和稀疏点，再训练一个固定点数的RGB高斯基线。深度数据继续保存，但当前RGB与深度标定未通过，不把深度位姿直接冒充彩色相机位姿，不加入错误深度监督。

设备核验：RTX4060 Laptop，8188MiB显存；D盘初始剩余约197GiB。训练速度和可承载场景尚未实测。

D:/cyberdog/tools/gaussian_runtime/.venv：Python3.11、pycolmap3.13、Pillow、NumPy、SciPy已安装，COLMAP接口可导入。

D:/cyberdog/tools/gaussian_runtime/train310：Python3.10已建立；PyTorch2.4.1 cu124与官方gsplat1.5.3+pt24cu124预编译包安装已启动。CUDA包较大，安装及真实渲染/梯度冒烟检查完成前，不视为GPU训练就绪。下载缓存和额外Python均设置在D盘。

## 已接入代码

1. tools/gaussian_dataset.py：从正式frames.jsonl导出照片及来源清单。原始深度保持原样。拒绝错误时间配对、变化的照片尺寸；去掉完全重复照片。
2. tools/gaussian_sfm.py：CPU SIFT特征、匹配、增量SfM、去畸变；输出估计的彩色相机模型。
3. tools/gaussian_train.py：试验性固定高斯点数训练，最多3万初始点，RGB损失，保留每第8个视角作图像检查，输出gaussians.npz、标准degree-0高斯PLY、留出视角图像和报告。尚未用真实绕物数据跑通训练，无增密、全局位姿优化、目标掩膜或深度监督。
4. tools/check_gaussian_runtime.py：真实CUDA光栅化及梯度检查，不能用仅import成功代替GPU可用。

现有20261101_160213_971260数据导出为evidence/gaussian_datasets/20261101_static_readiness：8张照片、估计深度相机位移8.4毫米，多视角初筛不通过，未训练。不是报错后将静止照片强行当作完整物体。

2项数据导出测试通过：重复静止照片不能伪装多视角、错误照片配对拒绝。SfM和真实高斯训练的完整流程仍需要新数据，不宣称已端到端验证。

## 新数据怎么采

环境准备确认后，由用户操控狗低速绕固定目标，始终朝向目标。建议首轮取得约40—80张清晰、重叠充分照片；30张只是初筛门槛，不保证成功。目标在照片中完整可见。优先采用移动一小段、停稳1—2秒的方式减少模糊，不是只在原地转头。

SfM初筛要求至少24张成功定位且占输入80%以上、至少100个稀疏点；这些是项目质量门槛，非理论精度保证。需要观察覆盖、重投影和留出图像表现。

由于背景纹理有助于SfM，照片保留全画面；最终“只显示中心目标”需要后续接入目标掩膜/裁剪。当前基线会重建场景，不能宣称已经实现目标专用高斯结果。

## 使用顺序

先使用原点云运行环境：

`D:/cyberdog/tools/reconstruction_runtime/.venv/Scripts/python.exe tools/gaussian_dataset.py <session> <new_dataset>`

再使用D盘gaussian_runtime/.venv运行：

`python tools/gaussian_sfm.py <dataset>`

GPU环境安装成功并通过check_gaussian_runtime后，使用train310：

`python tools/gaussian_train.py <dataset> <new_output> --steps 2000`

所有输出均要求新目录。当前没有一键网页训练或网页高斯查看器；不能把现有普通点云画布称为高斯渲染。

## 精度边界

RGB高斯模型通常改善新视角外观，但SfM尺度不自动具有米制含义，渲染好看也不代表尺寸正确。当前仍标记geometry_accuracy_verified=false。后续只有RGB/深度配准通过后，才比较深度误差或加入深度约束。

官方参考：

- https://github.com/nerfstudio-project/gsplat
- https://docs.gsplat.studio/whl/gsplat/
- https://docs.nerf.studio/nerfology/methods/splat.html
