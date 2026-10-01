# cam0 物料模型识别框架

本地夹爪相机的单路、双路调试入口已共用可配置的物料后端。
这是接入框架，不包含训练、已验证模型、三维定位或自动机械动作。
树莓派 serialPlotTest/pi-brain 是另一套目录/API，本文对应当前本地项目；
不能直接把这些文件覆盖到 pi-brain，也不表示已经部署到树莓派。

## 当前默认状态

[配置入口](../config/material.json)默认选择 model 后端，但权重为 null、
class_to_material 为空。此时不导入 Ultralytics、不下载或加载任何现有权重，
返回 MODEL_NOT_CONFIGURED，safe_to_pick 恒为 false。
仓库里已有的 best.pt、yolo26n.pt 等文件不会自动采用。

模型类别编号与比赛物料编号是两套标识，不使用 class_id + 1 自动换算。
编号确认以前不要填写臆测映射。
另请注意，cameras.json 中的 model 指摄像头硬件型号，不是网络权重；
训练权重应填在 material.json 中。

## 工作链路

cam0 单次采图 → 推理后端 → 原图尺寸的检测框、class_id、名称、置信度
→ 显式映射物料编号 → 置信度与框面积过滤 → 同位置目标连续确认
→ 选择目标物料 → 计算相对夹爪中心的像素偏差 → 输出视觉对准结果。

- 模型适配接口是 InferenceBackend.predict(frame)，返回 ModelDetection 序列。
  后续可增加其他推理引擎，不需要重写相机管理与对准层。
- 首个适配器使用 Ultralytics 的目标检测接口，只加载已存在的本地权重。
  模型类必须输出 boxes；只有分类标签或姿态结果的模型不能用于该对准流程。
- 当前不对模型画面额外做旧 HSV 路径的软件白平衡，训练和实机预处理应一致。
- 当前确认参数为连续 3 次、检测框 IoU 至少 0.3、确认间隔不超过 1 秒。
  目标消失、框位置不匹配、切换请求物料或推理错误都会重新确认。
- 同一类别同时出现多个目标时暂时返回 AMBIGUOUS，而不是任选一个自动抓取。
- 对准中心、像素容差仍取自 cameras.json：当前为 (320,240)、横纵 ±18 像素。
  这些只是占位标定值，模型接入不等于完成夹爪机械标定。
- safe_to_pick 仅表示调试用视觉条件，不直接发抓取指令，也不替代机械/运动安全检查。
- 配置 Stm32MaterialVisionPublisher 后，类别、置信度和像素位置交给 STM32；
  未对准也发送，不使用 safe_to_pick 门控。纠偏和抓取由 STM32 判断与执行。
  下发/接收入口与协议见 [STM32 物料数据说明](MATERIAL_STM32.md)。

## 以后需要填写什么

在 material.json 中填写：

1. model.weights：训练完成的本地目标检测权重。
   相对路径以 material.json 所在目录为基准，例如 ../weights/material.pt。
2. class_to_material：类别键为 0、1 等模型真实 class_id 的文本，
   每个映射项包含正整数 material_code 和可选 name。允许的框架编号不限定 1～6。
   不同类别不能映射到同一物料编号。
3. 根据验证集和实机结果调整 confidence_threshold、image_size、确认参数。

配置结构示意（保持未配置状态，不是可用模型）：

~~~json
{
  "backend": "model",
  "model": {
    "weights": null
  },
  "class_to_material": {}
}
~~~

修改配置后重启调试程序。模型有了、编号仍未确定时，可以只填写权重：
程序会显示原始模型类别、置信度和框，但返回 MAPPING_NOT_CONFIGURED，
不产生已编号物料观测，也不允许抓取。
如果映射仅填了一部分，画面出现任何未映射的高置信度类别，
整个当前帧返回 UNMAPPED_CLASS 并阻止抓取。

注意：现有任务码解析器仍使用原比赛 1～6 号规则。以后决定编号时，
需要同时确认任务码、统计、显示与任务层的编号规则；
本次没有在未知编号的情况下修改这些业务规则。

## 输入颜色顺序

Ultralytics 的 NumPy 图像输入要求 HWC、uint8、BGR。
Picamera2 的 RGB888 格式在 capture_array 中为 BGR 字节顺序，
因此当前 input_color_order=BGR；不要仅按格式名称猜测数组顺序。
若以后更换图像源输出 RGB，配置 RGB，适配器会转成连续内存的 BGR。

参考：[Ultralytics 预测接口](https://docs.ultralytics.com/modes/predict/)、
[Picamera2 格式映射源码](https://github.com/raspberrypi/picamera2/blob/main/picamera2/request.py)。

## 调试和测试

框架测试不需要训练模型或真实相机：

~~~bash
python3 -m unittest tests.test_material_model tests.test_gripper_material tests.test_dual_camera_vision -v
~~~

树莓派具备 Picamera2 和 OpenCV 时，检查 cam0 未配置状态：

~~~bash
python3 -m tools.debug_gripper --no-preview
python3 -m tools.debug_dual_camera --mode gripper
~~~

有本地桌面时，单路省略 --no-preview，双路添加 --preview。
这些调试程序都不会执行抓取动作。

需要原颜色识别对照时必须显式选择，不自动回退：

~~~bash
python3 -m tools.debug_gripper --material-backend color --no-preview
python3 -m tools.debug_dual_camera --mode gripper --material-backend color
~~~

训练完成后按需安装可选推理依赖：

~~~bash
python3 -m pip install -r requirements-model.txt
~~~

本次不安装推理依赖、不运行真实权重、不训练、不下载模型。
测试中的检测框和权重加载器均为模拟，不能代表树莓派帧率和识别准确率。

## 安全等待状态

| 状态 | 含义 |
|---|---|
| MODEL_NOT_CONFIGURED | 尚未配置训练权重 |
| MAPPING_NOT_CONFIGURED | 可以查看模型类别，但尚无物料编号映射 |
| UNMAPPED_CLASS | 当前帧存在未映射类别 |
| TARGET_NOT_MAPPED | 请求的物料编号没有对应类别 |
| TARGET_NOT_FOUND | 已映射目标在当前帧未出现 |
| INFERENCE_ERROR | 权重不存在、依赖缺失、加载/推理失败或输出格式错误 |
| AMBIGUOUS | 同一类别有多个物料，暂不支持自动选实例 |
| CONFIRMING_MATERIAL | 目标存在，但连续确认次数不足 |
| ALIGNING | 目标已确认，但未进入夹爪容差 |
| READY | 当前视觉条件满足；仍由上层安全控制决定机械动作 |
