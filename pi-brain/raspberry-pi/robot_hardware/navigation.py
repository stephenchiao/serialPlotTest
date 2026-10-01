"""从项目配置创建真实导航硬件链路。"""

from __future__ import annotations

import json
from pathlib import Path

from robot_control.dual_camera_vision import build_dual_camera_vision
from robot_control.navigation_runtime import build_navigation_runtime
from robot_hardware.stm32 import SerialLink


PROJECT_ROOT = Path(__file__).resolve().parents[1]


class NavigationHardwareConfigError(ValueError):
    pass


def validate_real_navigation_configuration() -> None:
    """拒绝用端口占位符、名义地图或未标定视觉参数启动电机。"""

    files = {
        name: json.loads((PROJECT_ROOT / "config" / filename).read_text(encoding="utf-8"))
        for name, filename in {
            "stm32": "stm32.json",
            "navigation": "navigation.json",
            "ops9": "ops9.json",
            "obstacle": "obstacle.json",
            "road": "road.json",
        }.items()
    }
    errors = []
    if "替换" in str(files["stm32"]["port"]):
        errors.append("STM32 串口设备名仍是占位符")
    if files["navigation"].get("nominal_map_requires_field_calibration", True):
        errors.append("场地图和点位尚未现场测量")
    if files["ops9"].get("calibration_required", True):
        errors.append("OPS9 起始坐标变换尚未标定")
    if files["obstacle"].get("calibration_required", True):
        errors.append("前视相机地面单应矩阵尚未标定")
    if files["road"].get("calibration_required", True):
        errors.append("灰/黄/白道路颜色阈值尚未标定")
    if errors:
        raise NavigationHardwareConfigError("；".join(errors))


def build_real_navigation():
    """创建可注入 ComponentBundle 的 navigator、motion 和 safety。

    如果道路颜色或相机单应矩阵仍未标定，本函数会直接拒绝创建，防止占位参数
    驱动车辆。
    """

    validate_real_navigation_configuration()
    stm32_path = PROJECT_ROOT / "config" / "stm32.json"
    stm32 = json.loads(stm32_path.read_text(encoding="utf-8"))
    link = SerialLink.from_config(stm32)
    vision = build_dual_camera_vision(
        camera_config_path=str(PROJECT_ROOT / "config" / "cameras.json"),
        color_config_path=str(PROJECT_ROOT / "config" / "color.json"),
        obstacle_config_path=str(PROJECT_ROOT / "config" / "obstacle.json"),
        road_config_path=str(PROJECT_ROOT / "config" / "road.json"),
        enable_navigation_perception=True,
    )
    return build_navigation_runtime(link, vision)
