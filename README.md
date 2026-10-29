# PICO 4 Ultra 遥操作 RealMan RML63 系列

将 PICO 右手柄的位姿和扳机输入映射到 RealMan RML63 系列六轴机械臂及 EG2-4C2 夹爪。项目提供两种控制入口：PC 侧 Placo 逆运动学后发送关节目标，以及发送笛卡尔位姿给控制器求解的位姿透传。实际使用的 URDF、控制器型号和工作坐标系需按现场配置核对。

> **状态：研究与联调原型。** 离线软件测试通过不等于现场安全认证。当前已在一套 RML63III-6FB 上观察到 SDK 指令返回成功和关节反馈变化，但也遇到过机械臂状态读取超时；不能据此认定通信和跟随性能已经稳定。首次使用前请从干运行开始，真机全程由操作者看护，实体急停须在手边。

## 功能范围

- 只读取右手柄；按住 Grip 接合平移/旋转离合，右扳机连续控制夹爪，A/B 分别发送打开/闭合命令。
- 摇杆、其他按键、力反馈、碰撞规划和自动回零尚未实现。
- `run_teleop.py` 默认干运行。只有增加 `--enable-motion`、通过配置检查，并在终端输入 `ENABLE` 才允许发送运动指令。
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

之后只编辑本机 `config/teleop.yaml`，填写控制器 IP，并在现场确认 XR 到机械臂基座的轴映射、末端工作空间和实体急停。轴映射要求正交；左右手系转换可使用行列式为 −1 的矩阵，姿态四元数通过矩阵共轭变换。`robot.model` 选择 IK 使用的 URDF 模型，`robot.controller_model` 必须填写控制器软件返回的 `product_version`。当前本机配置按用户指定使用 RML63II-6F-V 的 IK，同时匹配控制器返回的 RML63III-6FB；这只放宽型号身份校验，不会把 RML63III 的运动学模型接入 IK。两者若对应不同实物结构，不能据此认定运动学正确，必须先验证模型与现场机械臂一致。模板中 IP 为空、工作空间边界为空，两个现场确认项为 `false`；这些值不能直接用于真机运动。模板中的轴映射矩阵只是参考初值。

`config/teleop.yaml` 已加入 Git 忽略规则，适合填写本机 IP 和现场参数。若文件已存在，更新代码时请保留它，不要用示例文件覆盖。也可以通过 `--config /路径/配置.yaml` 指定配置；每条命令都需带上该参数。

即使配置中的 `calibration_confirmed` 和 `emergency_stop_confirmed` 都为 `true`，程序也不会自动进入真机模式；`--enable-motion` 和终端确认仍是必需的。

## 分阶段检查

先启动 PC Service，在头显内连接，并确认右手柄正在追踪、数据发送开关 `Send` 已打开。PICO 与 PC 需要网络互通；机械臂控制器也需能从 PC 访问。XR SDK 日志中的 `127.0.0.1:60061` 是本机 PC Service 地址，不是机械臂 IP。

以下命令均在项目目录、已激活的 Conda 环境中运行。使用自定义配置时，在每条命令后加 `--config /路径/配置.yaml`。

### 1. 检查 XR 输入

```bash
python scripts/check_xr_input.py
```

移动右手柄、按住 Grip、扣动扳机并按右手 A/B 键，观察位姿、`grip`、`trigger`、`A` 和 `B` 状态。输出中的 `base_xyz` 和 `base_quat_wxyz` 是按当前矩阵换算的结果；矩阵未现场核对前，不要据此操作真机方向。按 `Ctrl+C` 退出。

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

干运行读取 XR 和机械臂状态并计算目标，但 `arm_tx=0`、`gripper_tx=0`，不会发送动作。先小幅移动右手柄、按 Grip 并观察目标、IK 与安全状态；再按 A/B 确认日志分别记录 `open`/`close` 请求。若方向、目标位置或状态异常，停止并检查坐标标定和模型关系。干运行不能验证夹爪实际响应、碰撞、实际跟随速度或急停。

### 5. 真机运动

只有现场配置已核对、干运行结果合理、机器人状态读取稳定、周围空间清空且实体急停在手边，才启动。真机模式下 A/B 会直接向控制器发送 `open_position`/`close_position` 夹爪位置指令；首次验证先在夹爪周围无人、无工件的状态下点按，确认 A 是打开、B 是闭合，再正常夹取：

```bash
python scripts/run_teleop.py --enable-motion
```

程序显示机器人地址、当前关节角和运行参数后，还会要求精确输入：

```text
ENABLE
```

不输入该短语就不会启用运动。开始时应以低速、小范围逐项确认方向，并随时准备使用实体急停。

### 位姿透传版本（独立入口）

新增的位姿透传版本与上面的 PC 侧 IK 关节控制版本并存，不覆盖 `run_teleop.py`，也不需要另一份配置。它使用同一个 `config/teleop.yaml`、XR 坐标映射、Grip 离合、扳机/A/B 夹爪输入、工作空间、看门狗和 `ENABLE` 确认；真机启用前还会只读确认控制器处于真实模式且六轴全部使能。位姿模式向控制器发送笛卡尔位姿 `rm_movep_canfd`，不在 PC 上运行 Placo、不生成关节角指令。控制器收到位姿后仍会在控制器内部进行 IK 求解，因此这不是“系统完全不做 IK”。

先用干运行观察映射目标（不会发送机械臂或夹爪动作）：

```bash
python scripts/dry_run_pose_teleop.py
```

真机入口如下，仍需通过相同配置检查，并在终端输入 `ENABLE`：

```bash
python scripts/run_pose_teleop.py --enable-motion
```

位姿由控制器当前反馈作为 Grip 接合参考，位姿目标按当前 XR 到基座映射及 `scale_factor` 计算。控制器位姿反馈与 `rm_movep_canfd` 必须使用一致的当前工具坐标和工作坐标；若 XR 映射按机械臂基坐标标定，需确认控制器当前工作坐标系与基坐标一致。RealMan 将此接口定义为不经轨迹规划的位姿透传，所以此模式在主机增加了 0.05 m/s、15°/s 的笛卡尔目标速度限制；**真机模式**下，目标超前实测位姿超过 0.10 m 或 30° 时进入锁存 HOLD 并尝试慢停。干运行不会因机械臂没有执行预览目标而触发此跟踪偏差锁存。这些限制目前是代码内固定值，不是 YAML 项。此版本仍检查控制器当前关节软限位、目标位置工作空间和 XR 超时；关节步长、关节速度、关节跟踪误差这些依赖主机 IK 输出关节角的保护不适用于位姿透传，由控制器处理位姿到关节的求解与运动。限幅和工作空间都不能检测路径碰撞或自碰撞。首次真机验证应从很小的手柄位移开始，并观察控制器是否接受目标及实际姿态变化。

## 手柄操作

| 输入 | 功能 |
| --- | --- |
| 右手柄 Grip（默认阈值 0.9） | 按住时接合末端位姿离合；每次重新按下先记录参考位姿，不会在首帧跳动。松开后关节控制模式保持当前实测关节角，位姿模式保持当前实测位姿。松开不是急停。 |
| 右手柄平移 | 末端平移按 `teleop.scale_factor` 缩放。默认 0.5，即手柄移动 10 cm，目标移动约 5 cm。 |
| 右手柄旋转 | 以 Grip 接合时的姿态为参考映射相对旋转；小于 `rotation_deadband_deg` 的增量忽略，其余增量经四元数球面平滑。 |
| 右扳机 | 独立控制夹爪：0 对应 `open_position`，1 对应 `close_position`。启动时记录扳机基线，变化不足死区不会触发；夹爪命令最高 10 Hz。 |
| 右手 A 键 | 真机模式下按下时发送一次 `open_position` 命令；松开不改变夹爪状态。 |
| 右手 B 键 | 真机模式下按下时发送一次 `close_position` 命令；松开不改变夹爪状态。 |
| 摇杆及其他按键 | 当前未映射。 |

右手扳机的连续控制保持不变；A/B 键提供无需持续按住的单次开/关命令。干运行仅记录 A/B 请求，不发送夹爪命令；真机启用后才调用 RealMan 夹爪位置接口。A/B 同时按下时忽略按钮命令。设置 `gripper.enabled: false` 可禁止扳机和按钮产生的新夹爪命令。目标越界或 IK 连续失败时会锁存 HOLD 并阻止新的夹爪命令，需松开 Grip 后重新接合；A/B 也需松开再按才会产生新的边沿命令。

## 常用速度和保护参数

| 配置项 | 含义 | 当前默认值 |
| --- | --- | ---: |
| `teleop.scale_factor` | 手柄平移到末端平移的比例，不是关节速度 | 0.5 |
| `teleop.rotation_deadband_deg` | 忽略的小幅旋转角 | 1.0° |
| `teleop.rotation_smoothing_alpha` | 旋转目标的球面平滑系数；越小响应越慢 | 0.25 |
| `teleop.control_rate_hz` | 控制循环频率 | 50 Hz |
| `safety.max_joint_velocity_deg_s` | 软件限制的关节目标变化速度 | 40°/s |
| `safety.max_joint_step_deg` | 单次目标最大关节步长 | 1° |
| `safety.tracking_error_protection_enabled` | 跟踪误差自适应减速和持续超限保护的开关 | `true` |
| `safety.tracking_error_slowdown_start_deg` | 目标与实测角差超过此值后开始缩小新目标步幅 | 5° |
| `safety.max_joint_tracking_error_deg` | 最大跟踪误差；持续超限才进入保护 HOLD | 12° |
| `safety.tracking_error_stop_duration_ms` | 最大误差必须持续多久才进入保护 HOLD | 250 ms |
| `safety.xr_timeout_ms` | XR 数据停止更新后的软件超时 | 200 ms |

在 50 Hz 下，40°/s 对应每周期最多约 0.8° 的目标变化，仍受 1° 单步限制。降低 `scale_factor` 只会缩小手柄平移映射范围，不会直接调整关节角速度。控制循环频率也不是机械臂速度设置。

跟踪误差保护会在目标逐渐领先实测角时缩小新的关节目标步幅，让机械臂有机会跟上；误差超过最大值并持续达到设定时间后，进入锁存 HOLD 并尝试慢停，需松开 Grip 后重新接合。将 `tracking_error_protection_enabled` 设为 `false` 会关闭这项误差处理，但关节限位、目标步长/速度、工作空间、IK 与 XR 超时保护仍生效。此开关不代表关闭控制器自身保护。

## 关节范围与模型

| 关节 | 软限位范围 |
| --- | ---: |
| J1、J2、J4、J5 | −178°～178° |
| J3 | −178°～145° |
| J6 | −360°～360° |

IK 使用 `rml63_teleop/assets/rml63/robot_ik.urdf`，工具末端帧为 `gripper_eef`。仿真与可视化加载同目录 `robot_full.urdf`。模型在法兰与原夹爪之间加入 76 mm 圆柱连接段。位置内部单位为米，关节角为弧度；调用 RealMan SDK 时转换成度。软件工作空间只限制末端目标点，不检查运动路径碰撞或完整自碰撞。控制器 `Pose` 所用工具坐标系也未必等于 `gripper_eef`，标定时需核实参考点。

## 日志与故障排查

日志按配置频率写入 `logs/teleop_*.jsonl`，该目录已加入忽略规则，不应提交到公开仓库。日志包含运行模式、输入、当前和目标关节/末端状态、IK 与安全状态、动作发送次数及 SDK 返回码。`DRY_RUN` 模式下发送次数应保持为 0；SDK 返回码为 0 只表示接口接受了调用，不能单独证明机械臂已执行到位。

| 现象 | 检查方向 |
| --- | --- |
| 右手柄姿态全零或零四元数 | 确认头显/右手柄已唤醒并追踪、PC Service 正在运行、头显数据发送开关打开。启动时会等待有效数据；运行中数据失效会进入故障状态。 |
| XR watchdog 超时或时间戳不前进 | 检查无线网络、PC Service、头显休眠和 `Send` 状态；退出后恢复数据，再重新启动程序。 |
| 姿态有效但 EE 目标被工作空间拒绝 | 核对坐标变换、当前配置的工作空间与末端参考帧；不要盲目扩大边界。 |
| `arm_tx=0` 且状态为 HOLD | 检查是否是 `DRY_RUN`、Grip 是否接合、保护是否锁存，以及 IK/安全状态。 |
| 位姿模式收到控制器错误码 `4099` | RealMan 将 `4099`（`0x1003`）描述为目标处不可达/奇异点错误（[错误码说明](https://develop.realman-robotics.com/robot4th/json/errorList/)）。位姿目标是完整的位置和姿态；位置误差很小也不表示目标姿态可达。程序检测到控制器错误后会进入 FAULT 并尝试慢停。停止后先在控制器端查看并按设备流程处理错误，确认机械臂状态安全，再用示教器低速移离当前奇异构型；不要用相同手柄姿态立即重启。日志中的 `last_pose_rc=0` 只说明该次 API 调用返回成功，不代表控制器已完成 IK 或到达目标。 |
| 跟踪误差增加或进入 HOLD | 查看日志里的 `tracking_error_max_deg`、`tracking_scale`、`control_dt_s`，并用 `check_robot_status.py` 只读检查模式、各轴使能和反馈；确认通信稳定后再调整减速起点、持续时间或最大误差阈值。 |
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
