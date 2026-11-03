# 实时点云对齐与保存修复

## 修复与证据

- 建图中 vodom→base_link TF 停更，dog_pose 的 laser_odom 位姿仍发布。现在将完整位置与归一化四元数放入独立 TF Buffer，以深度采集时间插值，再与同时间的 base_link→相机变换组合。禁止使用最新位姿代替采集时位姿；定位/导航仍使用原 map TF。
- 等待对齐的帧不再被后续帧反复覆盖。新建图时清空位姿历史。
- 电脑后台自动保存对齐帧，无需浏览器开启。新建图名称及坐标系隔离；未知建图名称时不归档到旧地图。
- 保存目录：evidence/live_cloud_maps。session_*/frames.jsonl 保存对齐点云逐帧；session_*/map.json 保存体素累计；latest.json 为最新累计快照。快照约每2秒原子替换，写入失败由网页报告。逐帧数据可用于后续重建，突然断电仍可能损失未刷新系统缓冲数据。
- 浏览器刷新时从 /cloud/saved 恢复同名建图结果，不把不同坐标系的历史地图直接融合。电脑服务重启恢复已有快照；继续原建图使用现有 --resume-mapping 会话恢复机制。没有确认建图名称时不自动继续旧会话。
- 本次实际重载电脑服务后，恢复9276个体素，随后继续增长至9347；网页显示 laser_odom 对齐，无脚本异常。未发送行走命令，没有执行新一轮绕行。

## 验证

test_mapping_cloud_transform.py：带旋转的位姿组合、原始时间戳精确查询、缺位姿拒绝。
test_live_cloud_archive.py：逐帧保存、重复帧去重、磁盘快照恢复、地图名称隔离。
navigation_cloud_restore.cjs：重新打开页面恢复，同轮未对齐帧保留历史地图。
navigation_cloud_controls.cjs / navigation_cloud_fusion.cjs：原视图交互及累计回归。
navigation_live_views.cjs：真实机器人网页数据更新。

## 验收边界

旧的未保存地图无法恢复。新保存的是修复后收到的有效帧。坐标对齐和落盘已通过静态实机验证，运动后的重影、闭环漂移及绝对精度仍待人工遥控首次移动验收。灰雾仅为深度表面观测的XY投影提示，不能作为通行判据。
