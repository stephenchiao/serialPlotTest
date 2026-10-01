# 树莓派5双摄像头方案

## 固定角色

```text
摄像头1 -> gripper -> 物料颜色、中心位置和夹爪对准
摄像头2 -> front   -> 前向巡线、导航和起始二维码
```

当前接线为 `gripper -> CAM/DISP0`、`front -> CAM/DISP1`。这里的摄像头1/2是功能名称，
`camera_num` 是 Picamera2 设备编号，二者不要混淆。

统一配置位于 `config/cameras.json`。摄像头型号尚未确定，因此两个 `model`
当前都是 `null`；确定型号后只填写名称，不应把型号判断写进视觉算法。

`camera_num` 是 Picamera2 的枚举编号。当前按夹爪0、前置1配置；装好摄像头
以后执行：

```bash
rpicam-hello --list
```

核对传感器和编号。如果实际枚举顺序相反，只交换配置中的 `camera_num`，
`connector` 字段仍填写实际物理接口，不要交换视觉模块职责。

## 当前参数

前向摄像头：

```text
640 x 480, 30 FPS
固定在车体，朝前下方
只交给巡线、导航和二维码模块
```

抓取摄像头：

```text
640 x 480, 15 FPS
固定在夹爪，朝抓取中心
交给颜色、物料中心、加工放置和堆叠对准模块
```

这些是起始参数，实际值应根据摄像头型号、镜头视场、树莓派温度和并发负载
实测调整。

## cam0 模型识别与对准框架

当前默认后端由 config/material.json 选择为 model。由于训练模型和物料编号
尚未确定，权重与类别映射留空，返回 MODEL_NOT_CONFIGURED 且禁止抓取。
单路、双路调试入口使用同一物料工厂；没有任何按类别编号自动加一的映射。
模型框、置信度、连续确认和二维对准接口已经预留，
详见 [物料模型接入说明](MATERIAL_MODEL.md)。

旧 HSV 颜色识别保留为显式对照选项：调试时添加 --material-backend color。
以下颜色参数仅适用于该旧后端，不代表模型路径也执行颜色阈值判断。

## 旧颜色后端与通用对准结果

`robot_perception.material.GripperMaterialDetector` 复用 `config/color.json` 的
1～6号颜色规则，并输出：

- 物料编号与中英文颜色名；
- 物料中心和检测框；
- 相对夹爪中心的像素偏差；
- 归一化偏差；
- 多帧颜色是否确认；
- 是否进入对准容差；
- 当前是否允许抓取。

默认 `require_global_ready=false`。这是因为夹爪近距离画面可能只看到目标物料；
此时只要目标颜色经过多帧确认、位置已经对准且画面不歧义，就可以给上层返回
`safe_to_pick=True`。如果实际安装能稳定同时看到整批三件物料，可以改成 `true`。

`grip_center` 和 `alignment_tolerance_pixels` 是机械安装相关参数。摄像头或夹爪
重新安装后必须重新标定，不能沿用旧值。

## 调试命令

检查前向摄像头巡线：

```bash
python3 -m tools.debug_line
```

检查抓取摄像头全部颜色：

```bash
python3 -m tools.debug_color --preview
```

检查抓取摄像头对4号物料的颜色和对准偏差：

```bash
python3 -m tools.debug_gripper --material-backend color --target-code 4
```

同时检查两路相机：

```bash
python3 -m tools.debug_dual_camera --mode all
```

联合调试默认不创建Qt窗口，适合SSH运行；有树莓派桌面时添加 `--preview`。
摄像头2只在需要二维码时才运行扫码，且与巡线复用同一帧。这样可以减少CSI
采集请求和CPU负载。

调试窗口中的白色十字是配置的夹爪中心；目标框和十字之间的连线就是待消除的
视觉偏差。单路 SSH 无桌面时添加 `--no-preview`，从终端读取类别、物料编号、
置信度、偏差和安全等待状态；没有编号映射时只显示原始模型候选，不授权抓取。

## 接入状态机

真实组件工厂创建一个 `DualCameraManager`，并保持它是两路摄像头的唯一拥有者：

```text
manager.front   -> Navigator / TaskCodeReader
manager.gripper -> MaterialPerception
```

状态机只调用 `Navigator`、`TaskCodeReader` 和 `MaterialPerception`，不得直接导入
Picamera2。任一摄像头画面超时或采集异常时，真实 `SafetyMonitor` 应返回不安全，
由状态机进入 `SAFE_STOP`。

## 上车测试顺序

1. 断电安装两条CSI排线，再上电。
2. 用 `rpicam-hello --list` 核对两路设备。
3. 单独调试前向相机。
4. 单独调试抓取相机和六种颜色。
5. 标定抓取中心与像素容差。
6. 同时打开两路相机运行30分钟，观察掉帧、温度和供电。
7. 低速接入STM32底盘和机械臂。
8. 测试遮挡、断开或帧停止时的安全停车。
