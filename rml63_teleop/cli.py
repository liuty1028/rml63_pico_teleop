from __future__ import annotations

import argparse
import json
import logging
import time

import numpy as np

from .config import DEFAULT_CONFIG_PATH, load_config, validate_config
from .geometry import QUEST_REFERENCE_ROTATION, transform_controller_pose
from .realman_arm import RealManArm
from .runtime import TeleopRuntime
from .xr_client import XRTrackingError, XrClient


def _parser(description: str) -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=description)
    parser.add_argument("--config", default=str(DEFAULT_CONFIG_PATH))
    return parser


def check_robot_main() -> None:
    parser = _parser("Connect to the robot and read state; this command never moves it.")
    args = parser.parse_args()
    logging.basicConfig(level=logging.INFO, format="%(asctime)s %(levelname)s %(message)s")
    config = load_config(args.config)
    issues = validate_config(config)
    if not config["robot"].get("ip"):
        raise SystemExit("Set robot.ip in the config before checking the robot connection.")
    robot_cfg = config["robot"]
    robot = RealManArm(robot_cfg["ip"], robot_cfg["port"], robot_cfg["dof"], robot_cfg.get("controller_model", "RML63III-6FB"))
    try:
        info = robot.connect()
        state = robot.get_state()
        lower, upper = robot.get_joint_limits_rad()
        print(f"Software info: {info}")
        print(f"Joint degrees: {np.round(state.joint_deg, 4).tolist()}")
        print(f"Pose: {state.pose}")
        print(f"Controller joint limits (deg): min={np.rad2deg(lower).tolist()} max={np.rad2deg(upper).tolist()}")
        print(f"Motion readiness issues: {issues or 'none'}")
    finally:
        robot.disconnect()


def check_xr_main() -> None:
    parser = _parser("Read PICO right-controller input; this command never moves the robot.")
    args = parser.parse_args()
    config = load_config(args.config)
    rotation = np.asarray(config["mount"].get("xr_to_base_rotation") or QUEST_REFERENCE_ROTATION, dtype=float)
    xr = XrClient()
    last_tracking_warning = 0.0
    try:
        while True:
            try:
                frame = xr.read()
            except XRTrackingError as exc:
                now = time.monotonic()
                if now - last_tracking_warning >= 1.0:
                    print(f"右手柄暂时没有有效追踪数据：{exc}")
                    last_tracking_warning = now
                time.sleep(0.1)
                continue
            xyz_base, quat_base = transform_controller_pose(frame.right_pose, rotation)
            print(
                f"pose_xyzw={np.round(frame.right_pose, 4).tolist()} "
                f"base_xyz={np.round(xyz_base, 4).tolist()} "
                f"base_quat_wxyz={np.round(quat_base, 4).tolist()} "
                f"grip={frame.right_grip:.3f} trigger={frame.right_trigger:.3f} "
                f"A={frame.right_a} B={frame.right_b}"
            )
            time.sleep(0.1)
    except KeyboardInterrupt:
        pass
    finally:
        xr.close()


def check_robot_status_main() -> None:
    parser = _parser("只读查询控制器真实/仿真模式、六轴使能和夹爪状态，不发送动作、不修改模式。")
    args = parser.parse_args()
    config = load_config(args.config)
    robot_cfg = config["robot"]
    robot = RealManArm(robot_cfg.get("ip"), robot_cfg.get("port", 8080), robot_cfg.get("dof", 6), robot_cfg.get("controller_model", "RML63III-6FB"))
    try:
        info = robot.connect()
        print(f"控制器型号：{info['product_version']}")
        print("以下每项 return_code=0 才表示查询成功；run_mode.value 为 0=仿真、1=真实；joint_enable.value 为各轴使能状态。")
        print(json.dumps(robot.get_execution_status(), ensure_ascii=False, indent=2))
        print(f"当前实测关节角（度）：{np.round(robot.get_state().joint_deg, 3).tolist()}")
    finally:
        robot.disconnect()


def dry_run_main() -> None:
    parser = _parser("Run live XR + robot state + Placo IK without sending motion commands.")
    args = parser.parse_args()
    logging.basicConfig(level=logging.INFO, format="%(asctime)s %(levelname)s %(message)s")
    config = load_config(args.config)
    if not config["robot"].get("ip"):
        raise SystemExit("Set robot.ip before live dry-run; no robot motion commands will be sent.")
    TeleopRuntime(config, dry_run=True).run(enable_motion=False)


def teleop_main() -> None:
    parser = _parser("PICO 到 RML63 遥操作；默认干运行，只有显式启用才发送动作指令。")
    parser.add_argument("--enable-motion", action="store_true", help="通过配置检查和人工确认后允许真机运动")
    args = parser.parse_args()
    logging.basicConfig(level=logging.INFO, format="%(asctime)s %(levelname)s %(message)s")
    config = load_config(args.config)
    if not args.enable_motion:
        print("当前模式：DRY RUN（干运行），不会发送机械臂或夹爪动作。启用真机还需要 --enable-motion 参数和终端确认，配置中的两个 true 不会自动开启运动。")
    else:
        issues = validate_config(config, require_motion_ready=True)
        if issues:
            raise SystemExit("Motion is locked until configuration is complete:\n- " + "\n- ".join(issues))
    TeleopRuntime(config, dry_run=not args.enable_motion).run(enable_motion=args.enable_motion)


def pose_dry_run_main() -> None:
    """Preview Cartesian targets without sending arm or gripper commands."""
    from .pose_runtime import PoseTeleopRuntime

    parser = _parser("Run live XR + robot state using Cartesian pose pass-through without sending motion commands.")
    args = parser.parse_args()
    logging.basicConfig(level=logging.INFO, format="%(asctime)s %(levelname)s %(message)s")
    config = load_config(args.config)
    if not config["robot"].get("ip"):
        raise SystemExit("Set robot.ip before live dry-run; no robot motion commands will be sent.")
    PoseTeleopRuntime(config, dry_run=True).run(enable_motion=False)


def pose_teleop_main() -> None:
    """Run the separate pose pass-through teleoperation mode."""
    from .pose_runtime import PoseTeleopRuntime

    parser = _parser("PICO 到 RML63 笛卡尔位姿透传遥操作；默认干运行。")
    parser.add_argument("--enable-motion", action="store_true", help="通过配置检查和人工确认后允许真机运动")
    args = parser.parse_args()
    logging.basicConfig(level=logging.INFO, format="%(asctime)s %(levelname)s %(message)s")
    config = load_config(args.config)
    if not args.enable_motion:
        print("当前模式：位姿透传 DRY RUN，不会发送机械臂或夹爪动作。真机还需要 --enable-motion 和终端输入 ENABLE。")
    else:
        issues = validate_config(config, require_motion_ready=True)
        if issues:
            raise SystemExit("Motion is locked until configuration is complete:\n- " + "\n- ".join(issues))
    PoseTeleopRuntime(config, dry_run=not args.enable_motion).run(enable_motion=args.enable_motion)
