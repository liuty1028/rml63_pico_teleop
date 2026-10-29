# RML63II-6F-V + EG2-4C2 模型

`robot_full.urdf` 是真机工具坐标和仿真/可视化共用的完整模型；`robot_ik.urdf` 使用相同机械臂几何与工具偏置，并将夹爪关节固定，供 Placo 求解六个机械臂关节。机械臂来自 [RealManRobot/rm_models](https://github.com/RealManRobot/rm_models) 的 `RML63II-6F-V` 模型（源仓库提交 `0785b56b12a6259f0f4e5f258b5367c4d950d85e`）；本仓库仅保留所需 URDF、网格及上游许可证。原有 EG2-4C2 夹爪保持不变。模型也包含随新机械臂 URDF 提供的相机安装座和相机网格；安装座转轴固定在 0°。

- 机械臂关节：`joint_1` 至 `joint_6`
- IK 工具末端帧：`gripper_eef`
- 法兰与夹爪之间：76 mm 长、半径 29 mm 的圆柱连接件
- ROS `package://` 网格地址已改为相对 URDF 的本地路径，可在未安装 ROS 包的 Placo 和仿真环境中读取

仿真或可视化加载 `robot_full.urdf`。真机 IK 使用 `robot_ik.urdf`。模型坐标仍需依据实际安装方向和 TCP 标定；更换机器人型号后也应确认控制器报告的关节顺序和限位。
