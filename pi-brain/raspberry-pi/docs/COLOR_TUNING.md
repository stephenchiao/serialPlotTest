# 颜色识别调参与评测

## 文件职责

- `color_config.json`：唯一需要经常修改的参数文件。
- `color_config.py`：读取并检查配置，参数写错时立即报出具体位置。
- `color_detector.py`：摄像头无关的识别算法，可供主程序或评测程序调用。
- `color_detect.py`：树莓派摄像头、预览、状态JSON和安全退出。
- `color_evaluate.py`：读取带标签照片，计算准确率、精确率、召回率和F1。

## 最常调整的参数

### HSV颜色范围

在 `colors` 中修改每个颜色的 `ranges`：

```json
"ranges": [
  [[42, 145, 65], [85, 255, 255]]
]
```

两个三元组分别是 `[H下限, S下限, V下限]` 和 `[H上限, S上限, V上限]`。

- 漏检某种颜色：适当扩大H范围，或降低S/V下限。
- 把白色、灰色误判成绿色：提高绿色S下限。
- 两种颜色互相混淆：缩小它们相邻的H范围，并尽量让范围不重叠。

### 物体过滤

- `min_object_area`：提高可减少小噪点；降低可识别更小或更远的物料。
- `max_object_area_ratio`：提高可接受更大的近距离物料，但更容易把背景当物料。
- `min_fill_ratio`：提高会偏向完整、实心的色块。
- `min_solidity`：提高会排除凹陷、破碎和不规则色块。
- `min_aspect_ratio`、`max_aspect_ratio`：限制物体外接框长宽比。
- `edge_margin`：排除紧贴画面边缘的色块。
- `strict_*`：只作用于 `strict_shape_filter=true` 的颜色，当前用于黑色。

### 多帧稳定性

- `history_length=7`：保存最近7帧。
- `min_confirmations=4`：至少4帧出现才确认。

提高这两个值会减少瞬时误判，但识别响应更慢；降低会响应更快，但更容易闪烁。始终保证 `min_confirmations <= history_length`。

### 运行性能

- `config/cameras.json`中的`gripper.frame_size`：降低分辨率会提速，但小物料更容易漏检。
- `config/cameras.json`中的`gripper.fps`：提高帧率会提高时间分辨率，也会增加CPU负载。
- `morph_kernel_size`：3保留细节，5较均衡，7去噪更强但可能吃掉小物料。
- `buffer_count`：通常保持3，不建议为了提速降到1。

## 建立评测集

至少采集以下场景，建议总共50～100张：

1. 六种颜色分别出现在画面中心、边缘、近处和远处。
2. 三种物料同时出现的不同组合。
3. 转盘运动模糊和停止状态。
4. 比赛可能出现的亮、暗和偏色环境。
5. 没有物料、只有白色/灰色背景、赛道黑线的负样本。

把图片放入 `dataset`，复制 `color_labels.example.json` 为 `color_labels.json`，然后为每张图填写实际出现的颜色编号。

在树莓派桌面中可以直接采集与实时算法输入一致的图片：

```bash
cd ~/python
python3 -m tools.debug_color --preview \
  --capture-dir ~/python/data/color_dataset
```

把需要的场景放到摄像头前，在预览窗口中按 `S` 保存一张；按 `Q`退出。保存的是完成软件白平衡、但没有检测框和文字的干净画面。

标签示例：

```json
{
  "file": "dataset/frame_001.jpg",
  "expected_codes": [1, 2, 4]
}
```

负样本使用空列表：

```json
{
  "file": "dataset/no_material.jpg",
  "expected_codes": []
}
```

## 运行评测

```bash
cd ~/python
python3 -m tools.evaluate_color \
  ~/python/data/color_labels.json
```

保存详细报告：

```bash
cd ~/python
python3 -m tools.evaluate_color \
  ~/python/data/color_labels.json \
  --report ~/python/data/color_report.json
```

如果照片没有经过运行时的软件白平衡，可以试验：

```bash
cd ~/python
python3 -m tools.evaluate_color \
  ~/python/data/color_labels.json \
  --auto-white-balance
```

## 推荐调参流程

1. 固定一部分照片作为验证集，不参与日常观察调参，防止只适合少量样本。
2. 先调整HSV范围，再调整面积和形状参数，最后调整多帧参数。
3. 每次只改一组参数，保存一份独立配置，例如 `color_config_v2.json`。
4. 对同一标签集运行评测，比较完全正确率、各颜色精确率和召回率。
5. 精确率低表示误报多，应收紧范围或过滤；召回率低表示漏检多，应放宽范围或过滤。
6. 最后回到树莓派摄像头和真实转盘上，用 `--preview`检查动态效果。

运行不同配置无需修改代码：

```bash
cd ~/python
python3 -m tools.debug_color \
  --config ~/python/config/color_v2.json
```

注意：这里是对HSV和几何规则进行参数优化，不是训练神经网络模型。
