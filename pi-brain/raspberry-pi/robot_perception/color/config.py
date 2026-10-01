"""颜色识别配置的加载与校验。"""

from pathlib import Path
import json


DEFAULT_CONFIG_PATH = Path(__file__).resolve().parents[2] / "config" / "color.json"


class ConfigError(ValueError):
    """配置文件内容不合法。"""


def _require_positive(config, key, section):
    value = config.get(key)
    if not isinstance(value, (int, float)) or isinstance(value, bool) or value <= 0:
        raise ConfigError(f"{section}.{key} 必须是大于0的数字")


def _validate_hsv_vector(vector, label):
    if not isinstance(vector, list) or len(vector) != 3:
        raise ConfigError(f"{label} 必须是 [H, S, V]")

    limits = (179, 255, 255)
    for index, (value, limit) in enumerate(zip(vector, limits)):
        if not isinstance(value, int) or isinstance(value, bool) or not 0 <= value <= limit:
            channel = "HSV"[index]
            raise ConfigError(f"{label} 的 {channel} 必须在 0～{limit} 之间")


def validate_config(config):
    """尽早报告参数错误，避免比赛时静默使用错误阈值。"""
    required_sections = ("white_balance", "detection", "runtime", "colors")
    for section in required_sections:
        if section not in config:
            raise ConfigError(f"缺少配置项：{section}")

    white_balance = config["white_balance"]
    for key in ("enabled", "lock_camera_awb"):
        if not isinstance(white_balance.get(key), bool):
            raise ConfigError(f"white_balance.{key} 必须是 true 或 false")
    for key in (
        "calibration_frames",
        "sample_step",
        "min_brightness",
        "fallback_min_brightness",
        "max_channel_value",
        "max_channel_difference",
        "min_strict_pixels",
        "min_pixels",
        "gain_min",
        "gain_max",
    ):
        _require_positive(white_balance, key, "white_balance")
    if white_balance["gain_min"] > white_balance["gain_max"]:
        raise ConfigError("white_balance.gain_min 不能大于 gain_max")
    for key in ("calibration_frames", "sample_step", "min_strict_pixels", "min_pixels"):
        if not isinstance(white_balance[key], int) or isinstance(white_balance[key], bool):
            raise ConfigError(f"white_balance.{key} 必须是正整数")

    detection = config["detection"]
    for key in (
        "expected_color_count",
        "min_object_area",
        "strict_min_object_area",
        "max_object_area_ratio",
        "max_objects_per_color",
        "min_aspect_ratio",
        "max_aspect_ratio",
        "min_fill_ratio",
        "strict_min_fill_ratio",
        "min_solidity",
        "strict_min_solidity",
        "morph_kernel_size",
        "history_length",
        "min_confirmations",
        "state_valid_seconds",
    ):
        _require_positive(detection, key, "detection")

    edge_margin = detection.get("edge_margin")
    if (
        not isinstance(edge_margin, int)
        or isinstance(edge_margin, bool)
        or edge_margin < 0
    ):
        raise ConfigError("detection.edge_margin 不能小于0")
    if detection["min_aspect_ratio"] > detection["max_aspect_ratio"]:
        raise ConfigError("最小长宽比不能大于最大长宽比")
    if not 0 < detection["max_object_area_ratio"] <= 1:
        raise ConfigError("max_object_area_ratio 必须在0～1之间")
    for key in ("min_fill_ratio", "strict_min_fill_ratio", "min_solidity", "strict_min_solidity"):
        if not 0 < detection[key] <= 1:
            raise ConfigError(f"detection.{key} 必须在0～1之间")
    if detection["min_confirmations"] > detection["history_length"]:
        raise ConfigError("min_confirmations 不能大于 history_length")
    if detection["morph_kernel_size"] % 2 == 0:
        raise ConfigError("morph_kernel_size 应使用奇数，例如3、5、7")
    for key in (
        "expected_color_count",
        "max_objects_per_color",
        "morph_kernel_size",
        "history_length",
        "min_confirmations",
    ):
        if not isinstance(detection[key], int) or isinstance(detection[key], bool):
            raise ConfigError(f"detection.{key} 必须是正整数")

    runtime = config["runtime"]
    for key in ("print_interval", "state_write_interval"):
        _require_positive(runtime, key, "runtime")
    if not isinstance(runtime.get("state_file"), str) or not runtime["state_file"]:
        raise ConfigError("runtime.state_file 必须是非空路径")

    colors = config["colors"]
    if not isinstance(colors, list) or not colors:
        raise ConfigError("colors 必须是非空列表")

    seen_codes = set()
    for color_index, color in enumerate(colors):
        label = f"colors[{color_index}]"
        code = color.get("code")
        if not isinstance(code, int) or isinstance(code, bool) or code <= 0:
            raise ConfigError(f"{label}.code 必须是正整数")
        if code in seen_codes:
            raise ConfigError(f"颜色编号重复：{code}")
        seen_codes.add(code)

        for name_key in ("name", "cn_name"):
            if not isinstance(color.get(name_key), str) or not color[name_key]:
                raise ConfigError(f"{label}.{name_key} 必须是非空字符串")

        draw_color = color.get("draw_color")
        if (
            not isinstance(draw_color, list)
            or len(draw_color) != 3
            or any(
                not isinstance(value, int)
                or isinstance(value, bool)
                or not 0 <= value <= 255
                for value in draw_color
            )
        ):
            raise ConfigError(f"{label}.draw_color 必须是三个0～255整数")
        if not isinstance(color.get("strict_shape_filter"), bool):
            raise ConfigError(f"{label}.strict_shape_filter 必须是 true 或 false")

        ranges = color.get("ranges")
        if not isinstance(ranges, list) or not ranges:
            raise ConfigError(f"{label}.ranges 至少需要一个HSV范围")
        for range_index, hsv_range in enumerate(ranges):
            range_label = f"{label}.ranges[{range_index}]"
            if not isinstance(hsv_range, list) or len(hsv_range) != 2:
                raise ConfigError(f"{range_label} 必须包含下限和上限")
            lower, upper = hsv_range
            _validate_hsv_vector(lower, f"{range_label}下限")
            _validate_hsv_vector(upper, f"{range_label}上限")
            if any(low > high for low, high in zip(lower, upper)):
                raise ConfigError(f"{range_label}下限不能大于上限")

    if detection["expected_color_count"] > len(colors):
        raise ConfigError("expected_color_count 不能超过颜色规则数量")

    return config


def load_config(path=None):
    """从JSON读取配置；未指定路径时读取项目 ``config/color.json``。"""
    config_path = Path(path) if path else DEFAULT_CONFIG_PATH
    try:
        with config_path.open("r", encoding="utf-8") as file:
            config = json.load(file)
    except FileNotFoundError as error:
        raise ConfigError(f"找不到配置文件：{config_path}") from error
    except json.JSONDecodeError as error:
        raise ConfigError(
            f"配置文件JSON格式错误：第{error.lineno}行，第{error.colno}列"
        ) from error

    validate_config(config)
    config["_config_path"] = str(config_path.resolve())
    return config
