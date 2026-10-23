# 环境配置与复现

以下说明面向 Ubuntu 22.04、x86_64、Python 3.10。安装过程只准备软件，不连接或驱动机械臂。运行硬件需要另外安装 PC Service、PICO 端应用，并完成 README 中的现场配置。

## 1. 本机实际使用的版本

2026-10-23 读取 `xr_teleop` 环境得到：

| 组件 | 版本/情况 |
| --- | --- |
| 系统 | Ubuntu 22.04、Linux x86_64 |
| Python | 3.10.22 |
| NumPy | 2.2.6 |
| Placo | 0.10.1 |
| PyYAML | 6.0.3 |
| RealMan `robotic-arm` | 1.1.6 |
| `xrobotoolkit_sdk` | 1.0.2，旧版源码构建，模块为 CPython 3.10 Linux x86_64 扩展 |
| 控制器 | RML63III-6FB；现场曾读到 V1.7.4 固件 |

版本表记录本机情况，不代表所有操作系统、固件或 SDK 组合都经过验证。`environment.yml` 固定了直接运行依赖的版本，但没有锁定全部 Conda/Python 间接依赖，也没有写入本机环境路径。

## 2. 已有本项目环境

如果已经在这台电脑配置好 `xr_teleop`，无需重建环境：

```bash
conda activate xr_teleop
cd /你下载的仓库路径
python -m pip install -e . --no-deps --no-build-isolation
```

这里的 `--no-deps` 用于保留已经安装的 SDK 和依赖。它不适合直接拿来创建全新的环境。

## 3. 在新电脑创建基础环境

先安装 Miniconda 或 Anaconda。Ubuntu 的基础构建工具用于旧版 XR SDK 的源码编译：

```bash
sudo apt update
sudo apt install -y git build-essential
```

在包含 `environment.yml` 的本项目目录执行：

```bash
conda env create -f environment.yml
conda activate xr_teleop
```

如果同名环境已经存在，可以为新环境指定另一个名称，例如 `conda env create -n xr_teleop_demo -f environment.yml`。不要为了安装示例直接删除已有环境。

已有 Python 3.10 虚拟环境的同学也可以安装 `requirements.txt`，但需自行准备 C++ 运行库、构建工具以及下一节的 XR SDK：

```bash
python -m pip install -r requirements.txt
```

基础环境不包含 PC Service、头显 APK，也没有默认安装 XR Python SDK。下面两条 SDK 路径选择一条。

## 4A. 尽量复现本机 XR SDK 1.0.2

本机的 `xrobotoolkit_sdk 1.0.2` 是源码版。当前 PyPI 没有该版本页面，因此不能用 `pip install xrobotoolkit-sdk==1.0.2` 作为复现步骤。

本机存在的源码工作副本提交为：

- [Python 绑定](https://github.com/XR-Robotics/XRoboToolkit-PC-Service-Pybind/tree/c64ccf6acd577a333e03b66fafe8efeeceb511b1)：`c64ccf6acd577a333e03b66fafe8efeeceb511b1`
- [原生 SDK 所在仓库](https://github.com/XR-Robotics/XRoboToolkit-PC-Service/tree/85bac4dbc1fd5cef42c74a160d9c30aa3491f122)：`85bac4dbc1fd5cef42c74a160d9c30aa3491f122`

这两个提交用于记录本机源码基线；构建工具和间接依赖仍可能随新环境不同而变化。下面只构建客户端 SDK，不构建整个 Qt PC Service，也不会运行上游脚本中的环境删除操作。

保持基础 Conda 环境已激活，从本项目根目录开始：

```bash
mkdir -p third_party_src
git clone https://github.com/XR-Robotics/XRoboToolkit-PC-Service-Pybind.git third_party_src/XRoboToolkit-PC-Service-Pybind
git -C third_party_src/XRoboToolkit-PC-Service-Pybind checkout c64ccf6acd577a333e03b66fafe8efeeceb511b1

mkdir -p third_party_src/XRoboToolkit-PC-Service-Pybind/tmp
git clone https://github.com/XR-Robotics/XRoboToolkit-PC-Service.git third_party_src/XRoboToolkit-PC-Service-Pybind/tmp/XRoboToolkit-PC-Service
git -C third_party_src/XRoboToolkit-PC-Service-Pybind/tmp/XRoboToolkit-PC-Service checkout 85bac4dbc1fd5cef42c74a160d9c30aa3491f122

cd third_party_src/XRoboToolkit-PC-Service-Pybind
bash tmp/XRoboToolkit-PC-Service/RoboticsService/PXREARobotSDK/build.sh
mkdir -p lib include
cp tmp/XRoboToolkit-PC-Service/RoboticsService/PXREARobotSDK/build/libPXREARobotSDK.so lib/
cp tmp/XRoboToolkit-PC-Service/RoboticsService/PXREARobotSDK/PXREARobotSDK.h include/
cp -r tmp/XRoboToolkit-PC-Service/RoboticsService/PXREARobotSDK/nlohmann include/
export CMAKE_PREFIX_PATH="$(python -m pybind11 --cmakedir):${CMAKE_PREFIX_PATH:-}"
python -m pip install . --no-build-isolation
cd ../..
python -m pip install -e . --no-deps --no-build-isolation
```

每一步应成功后再执行下一步。如果构建目录中没有生成 `libPXREARobotSDK.so`，先排查构建输出，不能跳过它直接安装 Python 绑定。保留第三方源码目录；旧版扩展可能依赖其 `lib/` 目录中的动态库。

出现 `ImportError: libPXREARobotSDK.so` 时，可在本项目根目录临时指定库路径，再检查导入：

```bash
export LD_LIBRARY_PATH="$PWD/third_party_src/XRoboToolkit-PC-Service-Pybind/lib:${LD_LIBRARY_PATH:-}"
python -c "import xrobotoolkit_sdk; print(xrobotoolkit_sdk.__file__)"
```

上面的环境变量只影响当前终端；如每次启动都需要它，应在你自己的环境激活脚本中配置。本项目不会自动修改 `.bashrc` 或系统动态库目录。

## 4B. 使用官方当前提供的 PyPI 安装路径

官方 Python 绑定仓库现在提供 [PyPI 安装方式](https://github.com/XR-Robotics/XRoboToolkit-PC-Service-Pybind#installation)。本项目选装项固定为公开发布的 `1.1.0`：

```bash
python -m pip install -e ".[xr]" -c requirements.txt
```

这条命令安装本项目和 `xrobotoolkit-sdk==1.1.0`。**它与本机联调使用的源码版 1.0.2 不同，本项目没有完成该版本与现场 PC Service/头显组合的真机验证。** 使用它时先检查导入和手柄输入，再做只读设备检查。不要对正在联调的旧环境直接执行这条命令来升级 SDK。

如果没有适合操作系统/CPU/Python 的轮子，按官方仓库说明从源码构建；不要复制另一台电脑的 `.so` 到当前环境来绕过版本/架构问题。

## 5. 安装 PC Service 和头显端

Python 包只负责读取 PC Service，不能代替服务或头显应用：

1. 从 [XRoboToolkit PC Service](https://github.com/XR-Robotics/XRoboToolkit-PC-Service) 的发布说明安装与你的 Ubuntu 版本、CPU 架构匹配的程序。
2. 按 [XRoboToolkit 官方入门说明](https://github.com/XR-Robotics) 和 [Unity Client 说明](https://github.com/XR-Robotics/XRoboToolkit-Unity-Client) 安装 PICO 端应用。
3. 启动 PC Service，打开头显内的 XRoboToolkit，连接电脑对应的服务地址。
4. 在头显应用中确认控制器数据同步已启用，`Send` 开关已打开；仅看到设备连接不等于正在发送姿态。

PC Service 的常见 Linux 启动入口为 `/opt/apps/roboticsservice/runService.sh`，实际以所装版本为准。头显与电脑需要能互通；电脑与机械臂控制器也需要正确的网段和路由。SDK 日志中的 `127.0.0.1:60061` 是本机服务地址，不是机械臂 IP。

## 6. 只检查软件安装，不连接硬件

在项目根目录执行：

```bash
python -c "import numpy, yaml, placo, xrobotoolkit_sdk; from Robotic_Arm.rm_robot_interface import RoboticArm; print('依赖导入成功')"
python -m pip check
python scripts/run_teleop.py --help
python -m unittest discover -s tests -v
```

上述导入检查不调用 XR 的 `init()`，测试中的外部 SDK/XR 通信由模拟对象替代。硬件运行还要按 [README](../README.md) 复制配置、完成标定并逐步检查。

常见环境问题：

| 现象 | 处理方向 |
| --- | --- |
| `ModuleNotFoundError: xrobotoolkit_sdk` | 激活正确环境，并完成 4A 或 4B；基础 Conda 文件不安装它。 |
| 找不到 `Robotic_Arm` | 包名是 `robotic-arm`，导入名是 `Robotic_Arm`；用当前环境的 `python -m pip` 安装。 |
| 缺少 `GLIBCXX_*` | 检查 Conda 的 `libstdcxx-ng` 和动态库加载路径，不要随意替换系统库。 |
| `libPXREARobotSDK.so` 加载失败 | 检查旧版 SDK 的构建产物、架构和库路径。 |
| 依赖能导入，但输入全零/超时 | 检查 PC Service、头显追踪和 `Send` 开关；这不等于 Python 包安装失败。 |
