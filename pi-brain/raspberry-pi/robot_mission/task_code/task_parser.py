"""把二维码文本解析为可供任务状态机使用的结构化任务。"""

from argparse import ArgumentParser
from dataclasses import asdict, dataclass
import json
import re
from typing import Iterable, Tuple


TASK_CODE_PATTERN = re.compile(r"[0-9]{3}(?:\+[0-9]{3}){3}")
DEFAULT_MATERIAL_CODES = (1, 2, 3, 4, 5, 6)
DEFAULT_PROCESS_POSITIONS = (1, 2, 3)


class TaskCodeError(ValueError):
    """二维码文本不符合当前比赛任务码规则。"""


@dataclass(frozen=True)
class MaterialTask:
    """一件物料的抓取序号和加工位置。"""

    sequence: int
    material_code: int
    process_position: int


@dataclass(frozen=True)
class BatchTask:
    """一批三件物料的执行计划。"""

    batch_number: int
    pickup_order: Tuple[int, int, int]
    process_positions: Tuple[int, int, int]
    post_process_action: str
    materials: Tuple[MaterialTask, MaterialTask, MaterialTask]


@dataclass(frozen=True)
class CompetitionTask:
    """完整二维码对应的两批搬运任务。"""

    raw_code: str
    first_batch: BatchTask
    second_batch: BatchTask

    def to_dict(self):
        """转换为可以直接写入JSON日志或进程消息的字典。"""
        return asdict(self)


def _normalise_allowed_values(values: Iterable[int], label: str):
    normalised = tuple(values)
    if not normalised:
        raise ValueError(f"{label}不能为空")
    if any(
        not isinstance(value, int) or isinstance(value, bool)
        for value in normalised
    ):
        raise ValueError(f"{label}必须全部是整数")
    if len(set(normalised)) != len(normalised):
        raise ValueError(f"{label}不能包含重复值")
    return normalised


def _validate_material_group(group, label, allowed_codes):
    invalid = sorted(set(group) - set(allowed_codes))
    if invalid:
        raise TaskCodeError(
            f"{label}包含不支持的物料编号：{invalid}；"
            f"允许值为{list(allowed_codes)}"
        )
    if len(set(group)) != len(group):
        raise TaskCodeError(f"{label}不能包含重复的物料编号")


def _validate_position_group(group, label, allowed_positions):
    if len(allowed_positions) != 3:
        raise ValueError("当前三物料任务要求加工位置配置正好包含3个值")
    if set(group) != set(allowed_positions):
        raise TaskCodeError(
            f"{label}必须是{list(allowed_positions)}的一种排列，"
            f"实际为{list(group)}"
        )


def _build_batch(
    batch_number,
    pickup_order,
    process_positions,
    post_process_action,
):
    materials = tuple(
        MaterialTask(
            sequence=index,
            material_code=material_code,
            process_position=process_position,
        )
        for index, (material_code, process_position) in enumerate(
            zip(pickup_order, process_positions),
            start=1,
        )
    )
    return BatchTask(
        batch_number=batch_number,
        pickup_order=pickup_order,
        process_positions=process_positions,
        post_process_action=post_process_action,
        materials=materials,
    )


def parse_task_code(
    raw_code,
    valid_material_codes=DEFAULT_MATERIAL_CODES,
    valid_process_positions=DEFAULT_PROCESS_POSITIONS,
    require_same_material_set=True,
):
    """解析形如 ``452+321+254+312`` 的比赛任务码。

    四组数字依次解释为：第一批抓取顺序、第一批加工位置、
    第二批抓取顺序、第二批加工位置。当前记忆只明确第一批加工后
    还要进入暂存区，因此这里只记录后续阶段，不推断暂存位置映射。
    """
    if not isinstance(raw_code, str):
        raise TaskCodeError("任务码必须是字符串")

    code = raw_code.strip()
    if not TASK_CODE_PATTERN.fullmatch(code):
        raise TaskCodeError(
            "任务码格式应为四组三位数字，并用+连接，"
            "例如452+321+254+312"
        )

    material_codes = _normalise_allowed_values(
        valid_material_codes,
        "允许的物料编号",
    )
    process_positions = _normalise_allowed_values(
        valid_process_positions,
        "允许的加工位置",
    )
    groups = tuple(tuple(int(char) for char in group) for group in code.split("+"))
    first_pickup, first_positions, second_pickup, second_positions = groups

    _validate_material_group(first_pickup, "第一批抓取顺序", material_codes)
    _validate_material_group(second_pickup, "第二批抓取顺序", material_codes)
    _validate_position_group(first_positions, "第一批加工位置", process_positions)
    _validate_position_group(second_positions, "第二批加工位置", process_positions)

    if require_same_material_set and set(first_pickup) != set(second_pickup):
        raise TaskCodeError(
            "两批物料编号集合必须相同；"
            f"第一批为{list(first_pickup)}，第二批为{list(second_pickup)}"
        )

    return CompetitionTask(
        raw_code=code,
        first_batch=_build_batch(
            batch_number=1,
            pickup_order=first_pickup,
            process_positions=first_positions,
            post_process_action="temporary_storage",
        ),
        second_batch=_build_batch(
            batch_number=2,
            pickup_order=second_pickup,
            process_positions=second_positions,
            post_process_action="stack_on_matching_first_batch",
        ),
    )


def main():
    """提供一个最小命令行入口，便于在树莓派上独立验证。"""
    argument_parser = ArgumentParser(description="解析智能搬运比赛任务码")
    argument_parser.add_argument(
        "task_code",
        help="四组三位数字，例如452+321+254+312",
    )
    args = argument_parser.parse_args()

    try:
        task = parse_task_code(args.task_code)
    except TaskCodeError as error:
        argument_parser.error(str(error))

    print(json.dumps(task.to_dict(), ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
