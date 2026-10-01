"""使用带标签的照片评测当前颜色参数，不需要连接摄像头。"""

from collections import defaultdict
from pathlib import Path
import argparse
import json

import cv2
import numpy as np

from robot_perception.color.config import ConfigError, DEFAULT_CONFIG_PATH, load_config
from robot_perception.color.detector import (
    CompetitionColorDetector,
    apply_white_balance,
    build_white_balance_luts,
    estimate_white_balance,
)


def parse_arguments():
    parser = argparse.ArgumentParser(description="颜色识别离线正确率评测")
    parser.add_argument("labels", help="标签JSON文件")
    parser.add_argument(
        "--config",
        default=str(DEFAULT_CONFIG_PATH),
        help="需要评测的颜色参数文件",
    )
    parser.add_argument(
        "--auto-white-balance",
        action="store_true",
        help="对每张图片单独估计软件白平衡",
    )
    parser.add_argument("--report", help="可选：保存详细JSON报告")
    return parser.parse_args()


def load_samples(labels_path):
    try:
        with labels_path.open("r", encoding="utf-8") as file:
            document = json.load(file)
    except FileNotFoundError as error:
        raise ValueError(f"找不到标签文件：{labels_path}") from error
    except json.JSONDecodeError as error:
        raise ValueError(
            f"标签JSON格式错误：第{error.lineno}行，第{error.colno}列"
        ) from error

    samples = document.get("samples") if isinstance(document, dict) else None
    if not isinstance(samples, list) or not samples:
        raise ValueError("标签文件必须包含非空的 samples 列表")

    for index, sample in enumerate(samples):
        if not isinstance(sample, dict):
            raise ValueError(f"samples[{index}] 必须是对象")
        if not isinstance(sample.get("file"), str) or not sample["file"]:
            raise ValueError(f"samples[{index}].file 必须是图片路径")
        expected = sample.get("expected_codes")
        if (
            not isinstance(expected, list)
            or any(not isinstance(code, int) for code in expected)
        ):
            raise ValueError(f"samples[{index}].expected_codes 必须是整数列表")
    return samples


def read_image(path):
    """imdecode兼容包含中文的Windows文件路径。"""
    try:
        encoded = np.fromfile(path, dtype=np.uint8)
    except OSError:
        return None
    if encoded.size == 0:
        return None
    return cv2.imdecode(encoded, cv2.IMREAD_COLOR)


def safe_divide(numerator, denominator):
    return numerator / denominator if denominator else 0.0


def evaluate(config, labels_path, auto_white_balance=False):
    detector = CompetitionColorDetector(config)
    samples = load_samples(labels_path)
    label_directory = labels_path.parent
    valid_codes = {color["code"] for color in config["colors"]}
    statistics = defaultdict(lambda: {"tp": 0, "fp": 0, "fn": 0})
    details = []
    exact_matches = 0

    for index, sample in enumerate(samples):
        image_path = Path(sample["file"])
        if not image_path.is_absolute():
            image_path = label_directory / image_path
        image_path = image_path.resolve()

        frame = read_image(image_path)
        if frame is None:
            raise ValueError(f"无法读取图片：{image_path}")

        if auto_white_balance:
            gains = estimate_white_balance(frame, config["white_balance"])
            if gains is not None:
                frame = apply_white_balance(
                    frame,
                    build_white_balance_luts(gains),
                )

        detections, _ = detector.detect_candidates(frame)
        predicted = {item["code"] for item in detections}
        expected = set(sample["expected_codes"])

        unknown_codes = expected - valid_codes
        if unknown_codes:
            raise ValueError(
                f"samples[{index}]包含配置中不存在的颜色编号："
                f"{sorted(unknown_codes)}"
            )

        exact_match = predicted == expected
        exact_matches += int(exact_match)

        for code in valid_codes:
            if code in expected and code in predicted:
                statistics[code]["tp"] += 1
            elif code not in expected and code in predicted:
                statistics[code]["fp"] += 1
            elif code in expected and code not in predicted:
                statistics[code]["fn"] += 1

        details.append(
            {
                "file": str(image_path),
                "expected_codes": sorted(expected),
                "predicted_codes": sorted(predicted),
                "exact_match": exact_match,
            }
        )

    color_results = []
    for color in sorted(config["colors"], key=lambda item: item["code"]):
        code = color["code"]
        counts = statistics[code]
        precision = safe_divide(counts["tp"], counts["tp"] + counts["fp"])
        recall = safe_divide(counts["tp"], counts["tp"] + counts["fn"])
        f1 = safe_divide(2 * precision * recall, precision + recall)
        color_results.append(
            {
                "code": code,
                "name": color["name"],
                **counts,
                "precision": precision,
                "recall": recall,
                "f1": f1,
            }
        )

    return {
        "config": config["_config_path"],
        "labels": str(labels_path.resolve()),
        "sample_count": len(samples),
        "exact_matches": exact_matches,
        "exact_accuracy": exact_matches / len(samples),
        "colors": color_results,
        "samples": details,
    }


def print_report(report):
    print(
        f'整张图片完全正确：{report["exact_matches"]}/'
        f'{report["sample_count"]} '
        f'({report["exact_accuracy"]:.1%})'
    )
    print("编号  颜色          TP  FP  FN  精确率   召回率   F1")
    for item in report["colors"]:
        print(
            f'{item["code"]:>2}    '
            f'{item["name"]:<12} '
            f'{item["tp"]:>3} '
            f'{item["fp"]:>3} '
            f'{item["fn"]:>3} '
            f'{item["precision"]:>7.1%} '
            f'{item["recall"]:>7.1%} '
            f'{item["f1"]:>7.1%}'
        )

    mistakes = [sample for sample in report["samples"] if not sample["exact_match"]]
    if mistakes:
        print("\n识别不正确的图片：")
        for sample in mistakes:
            print(
                f'- {sample["file"]}：'
                f'应为{sample["expected_codes"]}，'
                f'识别为{sample["predicted_codes"]}'
            )


def main():
    args = parse_arguments()
    labels_path = Path(args.labels)
    try:
        config = load_config(args.config)
        report = evaluate(config, labels_path, args.auto_white_balance)
    except (ConfigError, ValueError) as error:
        print(f"评测错误：{error}")
        return 2

    print_report(report)
    if args.report:
        report_path = Path(args.report)
        report_path.parent.mkdir(parents=True, exist_ok=True)
        with report_path.open("w", encoding="utf-8") as file:
            json.dump(report, file, ensure_ascii=False, indent=2)
        print(f"详细报告已保存：{report_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
