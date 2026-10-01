"""任务码解析器的无硬件单元测试。"""

import unittest

from robot_mission.task_code import TaskCodeError, parse_task_code


class ParseTaskCodeTests(unittest.TestCase):
    def test_parse_known_example(self):
        task = parse_task_code("452+321+254+312")

        self.assertEqual(task.raw_code, "452+321+254+312")
        self.assertEqual(task.first_batch.pickup_order, (4, 5, 2))
        self.assertEqual(task.first_batch.process_positions, (3, 2, 1))
        self.assertEqual(task.second_batch.pickup_order, (2, 5, 4))
        self.assertEqual(task.second_batch.process_positions, (3, 1, 2))
        self.assertEqual(task.first_batch.materials[0].material_code, 4)
        self.assertEqual(task.first_batch.materials[0].process_position, 3)
        self.assertEqual(
            task.second_batch.post_process_action,
            "stack_on_matching_first_batch",
        )

    def test_strip_surrounding_whitespace(self):
        task = parse_task_code("  452+321+254+312\n")
        self.assertEqual(task.raw_code, "452+321+254+312")

    def test_reject_bad_format(self):
        with self.assertRaisesRegex(TaskCodeError, "格式"):
            parse_task_code("452-321-254-312")

    def test_reject_unsupported_material_code(self):
        with self.assertRaisesRegex(TaskCodeError, "不支持"):
            parse_task_code("452+321+257+312")

    def test_reject_duplicate_material_in_a_batch(self):
        with self.assertRaisesRegex(TaskCodeError, "重复"):
            parse_task_code("455+321+254+312")

    def test_reject_invalid_process_position_permutation(self):
        with self.assertRaisesRegex(TaskCodeError, "排列"):
            parse_task_code("452+311+254+312")

    def test_reject_different_material_sets_between_batches(self):
        with self.assertRaisesRegex(TaskCodeError, "必须相同"):
            parse_task_code("452+321+153+312")

    def test_allow_configurable_material_codes(self):
        task = parse_task_code(
            "789+123+978+321",
            valid_material_codes=(7, 8, 9),
        )
        self.assertEqual(task.first_batch.pickup_order, (7, 8, 9))


if __name__ == "__main__":
    unittest.main()
