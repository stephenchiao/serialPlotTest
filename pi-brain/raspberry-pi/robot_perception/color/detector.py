"""与摄像头解耦的颜色识别核心，可用于实时运行和离线评测。"""

from collections import Counter, deque
import time

import cv2
import numpy as np


def compile_color_rules(colors):
    """把JSON列表转换成适合OpenCV重复使用的NumPy数组。"""
    rules = []
    for color in colors:
        rules.append(
            {
                "code": color["code"],
                "name": color["name"],
                "cn_name": color["cn_name"],
                "ranges": tuple(
                    (
                        np.array(lower, dtype=np.uint8),
                        np.array(upper, dtype=np.uint8),
                    )
                    for lower, upper in color["ranges"]
                ),
                "draw_color": tuple(color["draw_color"]),
                "strict_shape_filter": color["strict_shape_filter"],
            }
        )
    return tuple(rules)


def estimate_white_balance(frame, config):
    """从一帧中的亮色、低色差区域估计BGR增益。"""
    step = int(config["sample_step"])
    sample = frame[::step, ::step].astype(np.float32)
    channel_max = sample.max(axis=2)
    channel_min = sample.min(axis=2)
    brightness = sample.mean(axis=2)

    neutral = (
        (brightness >= config["min_brightness"])
        & (channel_max <= config["max_channel_value"])
        & (
            (channel_max - channel_min)
            <= config["max_channel_difference"]
        )
    )

    if np.count_nonzero(neutral) < config["min_strict_pixels"]:
        neutral = (
            (brightness >= config["fallback_min_brightness"])
            & (channel_max <= config["max_channel_value"])
        )

    if np.count_nonzero(neutral) < config["min_pixels"]:
        return None

    channel_means = sample[neutral].mean(axis=0)
    target = float(channel_means.mean())
    gains = target / np.maximum(channel_means, 1.0)
    return np.clip(
        gains,
        config["gain_min"],
        config["gain_max"],
    ).astype(np.float32)


def calibrate_white_balance(picam2, config):
    """读取多帧，以各通道增益的中位数作为固定软件白平衡。"""
    if not config["enabled"]:
        return np.ones(3, dtype=np.float32)

    estimates = []
    for _ in range(int(config["calibration_frames"])):
        frame = picam2.capture_array("main")
        gains = estimate_white_balance(frame, config)
        if gains is not None:
            estimates.append(gains)

    if not estimates:
        return np.ones(3, dtype=np.float32)

    return np.median(np.stack(estimates), axis=0).astype(np.float32)


def build_white_balance_luts(gains):
    """把三个浮点增益预计算为256项查找表。"""
    values = np.arange(256, dtype=np.float32)
    return tuple(
        np.clip(values * float(gain), 0, 255).astype(np.uint8)
        for gain in gains
    )


def apply_white_balance(frame, luts):
    """利用查找表快速校正B、G、R通道。"""
    channels = cv2.split(frame)
    corrected = tuple(
        cv2.LUT(channel, lookup_table)
        for channel, lookup_table in zip(channels, luts)
    )
    return cv2.merge(corrected)


class CompetitionColorDetector:
    """颜色候选检测、多帧投票与安全状态判断。"""

    def __init__(self, config):
        self.config = config
        self.detection = config["detection"]
        self.rules = compile_color_rules(config["colors"])
        kernel_size = int(self.detection["morph_kernel_size"])
        self.morph_kernel = np.ones((kernel_size, kernel_size), dtype=np.uint8)
        self.history = deque(maxlen=int(self.detection["history_length"]))

    def reset_history(self):
        """清除多帧投票，切换任务或评测样本时使用。"""
        self.history.clear()

    def _create_color_mask(self, hsv, ranges):
        mask = None
        for lower, upper in ranges:
            partial_mask = cv2.inRange(hsv, lower, upper)
            mask = (
                partial_mask
                if mask is None
                else cv2.bitwise_or(mask, partial_mask)
            )

        mask = cv2.morphologyEx(
            mask,
            cv2.MORPH_OPEN,
            self.morph_kernel,
        )
        mask = cv2.morphologyEx(
            mask,
            cv2.MORPH_CLOSE,
            self.morph_kernel,
        )
        return mask

    def _valid_object_contours(self, mask, strict_shape_filter):
        contours, _ = cv2.findContours(
            mask,
            cv2.RETR_EXTERNAL,
            cv2.CHAIN_APPROX_SIMPLE,
        )

        height, width = mask.shape
        max_area = height * width * self.detection["max_object_area_ratio"]
        prefix = "strict_" if strict_shape_filter else ""
        min_area = self.detection[f"{prefix}min_object_area"]
        min_fill = self.detection[f"{prefix}min_fill_ratio"]
        min_solidity = self.detection[f"{prefix}min_solidity"]
        edge_margin = int(self.detection["edge_margin"])
        valid = []

        for contour in contours:
            area = cv2.contourArea(contour)
            if not min_area <= area <= max_area:
                continue

            box = cv2.boundingRect(contour)
            x, y, object_width, object_height = box
            if (
                x <= edge_margin
                or y <= edge_margin
                or x + object_width >= width - edge_margin
                or y + object_height >= height - edge_margin
            ):
                continue

            aspect = object_width / max(object_height, 1)
            fill_ratio = area / max(object_width * object_height, 1)
            hull_area = cv2.contourArea(cv2.convexHull(contour))
            solidity = area / max(hull_area, 1.0)

            if not (
                self.detection["min_aspect_ratio"]
                <= aspect
                <= self.detection["max_aspect_ratio"]
            ):
                continue

            if fill_ratio < min_fill or solidity < min_solidity:
                continue

            valid.append((box, area, fill_ratio, solidity))

        valid.sort(key=lambda item: item[1], reverse=True)
        return valid[: int(self.detection["max_objects_per_color"])]

    def detect_candidates(self, frame, collect_masks=False):
        """只处理当前帧，不做时间投票；离线评测使用这个接口。"""
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        detections = []
        masks = []

        for rule in self.rules:
            mask = self._create_color_mask(hsv, rule["ranges"])
            if collect_masks:
                masks.append((mask, rule["draw_color"]))

            for box, area, fill_ratio, solidity in self._valid_object_contours(
                mask,
                rule["strict_shape_filter"],
            ):
                x, y, object_width, object_height = box
                detections.append(
                    {
                        "code": int(rule["code"]),
                        "name": rule["name"],
                        "cn_name": rule["cn_name"],
                        "center": [
                            int(x + object_width // 2),
                            int(y + object_height // 2),
                        ],
                        "box": [
                            int(x),
                            int(y),
                            int(object_width),
                            int(object_height),
                        ],
                        "area": round(float(area), 1),
                        "fill_ratio": round(float(fill_ratio), 3),
                        "solidity": round(float(solidity), 3),
                        "draw_color": rule["draw_color"],
                    }
                )

        detections.sort(key=lambda item: item["center"][0])
        return detections, masks

    def update_state(self, detections):
        """把当前帧候选加入历史投票，计算比赛安全状态。"""
        current_codes = {item["code"] for item in detections}
        self.history.append(current_codes)

        vote_counts = Counter(
            code
            for frame_codes in self.history
            for code in frame_codes
        )
        confirmed_codes = sorted(
            code
            for code, count in vote_counts.items()
            if count >= self.detection["min_confirmations"]
        )
        confirmed_set = set(confirmed_codes)
        expected_count = int(self.detection["expected_color_count"])

        if len(confirmed_codes) > expected_count:
            status = "AMBIGUOUS"
        elif len(confirmed_codes) < expected_count:
            status = "SEARCHING"
        elif current_codes == confirmed_set:
            status = "READY"
        else:
            status = "HOLD"

        for item in detections:
            item["confirmed"] = item["code"] in confirmed_set

        return {
            "timestamp": round(time.time(), 3),
            "valid_for_seconds": self.detection["state_valid_seconds"],
            "status": status,
            "safe_to_pick": status == "READY",
            "expected_color_count": expected_count,
            "confirmed_color_codes": confirmed_codes,
            "detections": detections,
        }

    def detect(self, frame, collect_masks=False):
        """实时入口：单帧候选检测后再进行历史投票。"""
        detections, masks = self.detect_candidates(frame, collect_masks)
        return self.update_state(detections), masks


def draw_preview(frame, state, masks):
    """生成标注图和综合色罩，只供人工调参。"""
    mask_preview = np.zeros_like(frame)
    for mask, draw_color in masks:
        mask_preview[mask > 0] = draw_color

    for detection in state["detections"]:
        x, y, object_width, object_height = detection["box"]
        center_x, center_y = detection["center"]
        draw_color = tuple(detection["draw_color"])
        box_color = (255, 255, 255) if detection["code"] == 5 else draw_color
        thickness = 2 if detection["confirmed"] else 1
        label = f'{detection["code"]} {detection["name"]}'

        cv2.rectangle(
            frame,
            (x, y),
            (x + object_width, y + object_height),
            box_color,
            thickness,
        )
        cv2.circle(frame, (center_x, center_y), 4, box_color, -1)
        cv2.putText(
            frame,
            label,
            (x, max(18, y - 6)),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.5,
            box_color,
            2,
        )

    status_color = (0, 255, 0) if state["safe_to_pick"] else (0, 165, 255)
    cv2.putText(
        frame,
        f'STATE: {state["status"]}',
        (8, 22),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.6,
        status_color,
        2,
    )
    return mask_preview


def serializable_state(state):
    """移除只用于OpenCV绘图的字段，使结果适合JSON输出。"""
    result = dict(state)
    result["detections"] = []
    for detection in state.get("detections", []):
        item = dict(detection)
        item.pop("draw_color", None)
        result["detections"].append(item)
    return result
