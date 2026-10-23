# PICO 4 Ultra 遥操作 RealMan RML63III-6FB

将 PICO 右手柄的位姿和扳机输入映射到 RealMan RML63III-6FB 六轴机械臂及 EG2-4C2 夹爪。项目包含 XR 输入、坐标变换、Placo 逆运动学、RealMan Python SDK 适配、软件限位和运行日志。

> **状态：研究与联调原型。** 离线软件测试通过不等于现场安全认证。当前已在一套 RML63III-6FB 上观察到 SDK 指令返回成功和关节反馈变化，但也遇到过机械臂状态读取超时；不能据此认定通信和跟随性能已经稳定。首次使用前请从干运行开始，真机全程由操作者看护，实体急停须在手边。

## 功能范围

- 只读取右手柄；按住 Grip 接合平移/旋转离合，右扳机单独控制夹爪。
- 摇杆、其他按键、力反馈、碰撞规划和自动回零尚未实现。
- `run_teleop.py` 默认干运行。只有增加 `--enable-motion`、通过配置检查，并在终端输入 `ENABLE RML63 MOTION` 才允许发送运动指令。
- 干运行仍连接 XR 和机械臂并读取实时状态，但不发送机械臂或夹爪动作。
- 每次启动读取控制器关节软限位并与 URDF 限位取交集；异常输入、跟踪超时、目标越界和 IK 失败会阻止或停止继续发送指令。软件慢停不能替代实体急停。

## 目录

```text
config/teleop.example.yaml       可公开的安全配置模板
docs/ENVIRONMENT.md              Conda、依赖、XR SDK 和 PC Service 安装说明
rml63_teleop/assets/rml63/       IK 与完整 URDF 模型
rml63_teleop/defaults/           随包提供的锁定默认配置
scripts/                         分阶段检查和运行入口
tests/                           离线软件测试
third_party/notices/             上游样例许可证副本
environment.yml                  Conda 基础环境
requirements.txt                 Python 核心依赖版本
```

## 环境和安装

运行环境说明见[环境配置与复现](docs/ENVIRONMENT.md)。项目当前以 Ubuntu 22.04、x86_64、Python 3.10 为基准。新电脑可按该文档创建 Conda 环境并选择 XR SDK 安装路径；若已配置本项目环境，可执行：

```bash
conda activate xr_teleop
cd /你的仓库路径/rml63_pico_teleop
python -m pip install -e . --no-deps --no-build-isolation
```

还需要单独安装并运行 XRoboToolkit PC Service、PICO 端应用，以及 RealMan 官方 `robotic-arm` SDK。Python SDK 不能代替 PC Service 或头显应用。其他 XRoboToolkit 示例仓库不是运行本项目的 Python 依赖；URDF 已包含在本仓库中。运行不需要 ROS。

## 配置

首次克隆时复制公开模板：

```bash
cp config/teleop.example.yaml config/teleop.yaml
```

之后只编辑本机 `config/teleop.yaml`，填写控制器 IP，并在现场确认 XR 到机械臂基座的旋转方向、末端工作空间和实体急停。模板中 IP 为空、工作空间边界为空，两个现场确认项为 `false`；这些值不能直接用于真机运动。模板中的旋转矩阵只是参考初值。

`config/teleop.yaml` 已加入 Git 忽略规则，适合填写本机 IP 和现场参数。若文件已存在，更新代码时请保留它，不要用示例文件覆盖。也可以通过 `--config /路径/配置.yaml` 指定配置；每条命令都需带上该参数。

即使配置中的 `calibration_confirmed` 和 `emergency_stop_confirmed` 都为 `true`，程序也不会自动进入真机模式；`--enable-motion` 和终端确认仍是必需的。

## 分阶段检查

先启动 PC Service，在头显内连接，并确认右手柄正在追踪、数据发送开关 `Send` 已打开。PICO 与 PC 需要网络互通；机械臂控制器也需能从 PC 访问。XR SDK 日志中的 `127.0.0.1:60061` 是本机 PC Service 地址，不是机械臂 IP。

以下命令均在项目目录、已激活的 Conda 环境中运行。使用自定义配置时，在每条命令后加 `--config /路径/配置.yaml`。

### 1. 检查 XR 输入

```bash
python scripts/check_xr_input.py
```

移动右手柄、按住 Grip、扣动扳机，观察位姿与 `grip`、`trigger` 数值。输出中的 `base_xyz` 和 `base_quat_wxyz` 是按当前矩阵换算的结果；矩阵未现场核对前，不要据此操作真机方向。按 `Ctrl+C` 退出。

### 2. 检查机械臂连接

在配置中填写控制器 IP 后运行：

```bash
python scripts/check_robot_connection.py
```

该脚本只读取型号、关节角、位姿和控制器关节软限位，不发送运动命令，也不修改控制器设置。状态读取成功并不代表运动链路已验证。

### 3. 查看控制器运行状态

```bash
python scripts/check_robot_status.py
```

只读查询真实/仿真模式、关节使能和夹爪状态，不切换模式、不上使能、不发送动作。只有每项 `return_code=0` 时，才可使用对应的查询值。

### 4. 干运行遥操作

```bash
python scripts/dry_run_teleop.py
```

干运行读取 XR 和机械臂状态并计算目标，但 `arm_tx=0`、`gripper_tx=0`，不会发送动作。先小幅移动右手柄、按 Grip 并观察目标、IK 与安全状态。若方向、目标位置或状态异常，停止并检查坐标标定和模型关系。干运行不能验证碰撞、实际跟随速度或急停。

### 5. 真机运动

只有现场配置已核对、干运行结果合理、机器人状态读取稳定、周围空间清空且实体急停在手边，才启动：

```bash
python scripts/run_teleop.py --enable-motion
```

程序显示机器人地址、当前关节角和运行参数后，还会要求精确输入：

```text
ENABLE RML63 MOTION
```

不输入该短语就不会启用运动。开始时应以低速、小范围逐项确认方向，并随时准备使用实体急停。

## 手柄操作

| 输入 | 功能 |
| --- | --- |
| 右手柄 Grip（默认阈值 0.9） | 按住时接合末端位姿离合；每次重新按下先记录参考位姿，不会在首帧跳动。松开后发送当前实测关节角作为保持目标。松开不是急停。 |
| 右手柄平移 | 末端平移按 `teleop.scale_factor` 缩放。默认 0.5，即手柄移动 10 cm，目标移动约 5 cm。 |
| 右手柄旋转 | 映射末端旋转增量；当前没有独立旋转缩放参数。 |
| 右扳机 | 独立控制夹爪：0 对应 `open_position`，1 对应 `close_position`。启动时记录扳机基线，变化不足死区不会触发；夹爪命令最高 10 Hz。 |
| 摇杆及其他按键 | 当前未映射。 |

夹爪通常不受 Grip 离合控制；设置 `gripper.enabled: false` 可禁止新夹爪命令。目标越界或 IK 连续失败时会锁存 HOLD 并阻止新的夹爪命令，需松开 Grip 后才能重新接合。

## 常用速度和保护参数

| 配置项 | 含义 | 当前默认值 |
| --- | --- | ---: |
| `teleop.scale_factor` | 手柄平移到末端平移的比例，不是关节速度 | 0.5 |
| `teleop.control_rate_hz` | 控制循环频率 | 50 Hz |
| `safety.max_joint_velocity_deg_s` | 软件限制的关节目标变化速度 | 20°/s |
| `safety.max_joint_step_deg` | 单次目标最大关节步长 | 1° |
| `safety.max_joint_tracking_error_deg` | 指令与实测关节角允许的最大差；超过即故障，不会自动等待机械臂追上 | 5° |
| `safety.xr_timeout_ms` | XR 数据停止更新后的软件超时 | 200 ms |

在 50 Hz 下，20°/s 对应每周期最多约 0.4° 的目标变化；若降为 10°/s，则约 0.2°/周期。降低速度上限会让关节目标变化更慢；降低 `scale_factor` 只会缩小手柄平移映射范围，不会直接调整关节角速度。控制循环频率也不是机械臂速度设置。

跟踪误差阈值是软件保护，不是机械臂的物理限速。若频繁报 `joint command is too far ahead of measured joints`，先检查机器人状态、通信稳定性、实际跟随和速度设置。不要仅为消除错误而大幅增加该阈值。当前版本不支持在误差过大时自动等候追上后继续。

## 关节范围与模型

| 关节 | 软限位范围 |
| --- | ---: |
| J1、J2、J4、J5 | −178°～178° |
| J3 | −178°～145° |
| J6 | −360°～360° |

IK 使用 `rml63_teleop/assets/rml63/robot_ik.urdf`，工具末端帧为 `gripper_eef`。完整模型为同目录 `robot_full.urdf`。位置内部单位为米，关节角为弧度；调用 RealMan SDK 时转换成度。软件工作空间只限制末端目标点，不检查运动路径碰撞或完整自碰撞。控制器 `Pose` 所用工具坐标系也未必等于 `gripper_eef`，标定时需核实参考点。

## 日志与故障排查

日志按配置频率写入 `logs/teleop_*.jsonl`，该目录已加入忽略规则，不应提交到公开仓库。日志包含运行模式、输入、当前和目标关节/末端状态、IK 与安全状态、动作发送次数及 SDK 返回码。`DRY_RUN` 模式下发送次数应保持为 0；SDK 返回码为 0 只表示接口接受了调用，不能单独证明机械臂已执行到位。

| 现象 | 检查方向 |
| --- | --- |
| 右手柄姿态全零或零四元数 | 确认头显/右手柄已唤醒并追踪、PC Service 正在运行、头显数据发送开关打开。启动时会等待有效数据；运行中数据失效会进入故障状态。 |
| XR watchdog 超时或时间戳不前进 | 检查无线网络、PC Service、头显休眠和 `Send` 状态；退出后恢复数据，再重新启动程序。 |
| 姿态有效但 EE 目标被工作空间拒绝 | 核对坐标变换、当前配置的工作空间与末端参考帧；不要盲目扩大边界。 |
| `arm_tx=0` 且状态为 HOLD | 检查是否是 `DRY_RUN`、Grip 是否接合、保护是否锁存，以及 IK/安全状态。 |
| 跟踪误差超限 | 先用 `check_robot_status.py` 只读检查运行模式、各轴使能和反馈；再检查速度、通信和目标跟随，不要直接放宽阈值。 |
| `rm_get_current_arm_state` 返回 `-2` | 表示 SDK 未能在该次调用取得有效反馈，可能与通信/接收超时有关。检查同一台 PC 到控制器的网络和只读状态查询。若机械臂仍在运动而软件慢停也失败，无法确认已停止时使用实体急停。 |

运行时输入或状态异常会触发软件故障处理并尝试慢停；网络阻塞或 SDK 无响应时，软件停止命令也可能无法送达。始终由操作者监视机械臂，实体急停保持可及。

## 软件验证

```bash
python -m unittest discover -s tests -v
python -m compileall -q rml63_teleop scripts tests
```

测试使用模拟 XR/SDK，不驱动真实设备。已验证模型关节范围、坐标变换、安全门限和若干离线正解对照；测试通过不能代替现场安装、碰撞检查或真机验收。

## 发布到 GitHub 前

- 提交 `config/teleop.example.yaml`，不要提交本机 `config/teleop.yaml`、日志、SDK 二进制、头显安装包或本机环境目录。
- 项目 `.gitignore` 已排除本机配置和日志。首次发布前用 `git status --short` 检查待提交文件，确认没有 IP、个人路径、运行日志或控制器信息。
- 项目中 XRoboToolkit 上游样例的许可证副本位于 `third_party/notices/`。机械臂模型/网格的来源与适用许可需按 [模型目录说明](rml63_teleop/assets/rml63/README.md) 核实；不要因为上游样例带有 MIT 许可证，就默认本仓库全部模型资产都适用该许可。
- 当前仓库尚未声明本项目整体许可证。公开前请根据你对自有代码及模型资产的授权情况决定是否添加项目许可证，并保留上游归属说明。

XRoboToolkit 安装细节和固定源码提交见[环境复现文档](docs/ENVIRONMENT.md)。安全复核记录见[真机前复核记录](PRE_FLIGHT_REVIEW.md)。
