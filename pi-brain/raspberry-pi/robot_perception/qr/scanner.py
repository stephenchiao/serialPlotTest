"""基于 OpenCV 的二维码扫描与相机读取适配器。"""

from typing import Any, Optional, Tuple


class QRCodeScannerError(RuntimeError):
    """二维码扫描依赖缺失或初始化失败。"""


def _load_cv2():
    try:
        import cv2
    except ImportError as error:
        raise QRCodeScannerError(
            "当前环境没有安装 OpenCV；请安装包含 QRCodeDetector 的 opencv-python"
        ) from error
    return cv2


class QRCodeScanner:
    """从单帧图像中解码一个或多个二维码。

    扫描器不持有摄像头，只接收图像帧，因此可以和巡线、颜色识别共用由
    ``robot_hardware.camera.PiCamera`` 管理的同一个摄像头。
    """

    def __init__(self, detector: Optional[Any] = None, enhance_fallback: bool = True):
        self._cv2 = None
        if detector is None:
            self._cv2 = _load_cv2()
            detector = self._cv2.QRCodeDetector()
        self.detector = detector
        self.enhance_fallback = bool(enhance_fallback)

    def decode(self, frame) -> Tuple[str, ...]:
        """返回当前帧中所有非空二维码文本，按首次出现顺序去重。"""
        if frame is None or not hasattr(frame, "shape") or len(frame.shape) < 2:
            raise ValueError("frame 必须是有效图像")

        decoded = self._decode_variant(frame)
        if not decoded and self.enhance_fallback:
            decoded = self._decode_enhanced(frame)

        return tuple(dict.fromkeys(
            value.strip() for value in decoded
            if isinstance(value, str) and value.strip()
        ))

    def _decode_variant(self, frame) -> Tuple[str, ...]:
        decoded = []
        decode_multi = getattr(self.detector, "detectAndDecodeMulti", None)
        if decode_multi is not None:
            result = decode_multi(frame)
            if isinstance(result, tuple) and len(result) >= 2 and result[0]:
                decoded.extend(result[1] or ())

        if decoded:
            return tuple(decoded)

        result = self.detector.detectAndDecode(frame)
        if isinstance(result, tuple) and result and result[0]:
            return (result[0],)
        return ()

    def _decode_enhanced(self, frame) -> Tuple[str, ...]:
        """原图失败后使用灰度和局部对比度增强图重试。"""
        cv2 = self._cv2 or _load_cv2()
        if len(frame.shape) == 2:
            gray = frame
        elif frame.shape[2] == 4:
            gray = cv2.cvtColor(frame, cv2.COLOR_RGBA2GRAY)
        else:
            # RGB/BGR 的灰度权重略有差异，但均不影响黑白二维码结构。
            gray = cv2.cvtColor(frame, cv2.COLOR_RGB2GRAY)

        decoded = self._decode_variant(gray)
        if decoded:
            return decoded

        enhanced = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8)).apply(gray)
        return self._decode_variant(enhanced)


class CameraTaskCodeReader:
    """实现运行时 ``TaskCodeReader`` 接口的非阻塞相机适配器。

    每次 ``read_task_code`` 只采集一帧。相同文本连续出现指定次数后才返回，
    防止运动模糊或远处图案造成单帧误识别。若同一帧出现多个不同二维码，
    将其视为歧义并继续等待。
    """

    def __init__(
        self,
        camera: Any,
        scanner: Optional[QRCodeScanner] = None,
        required_confirmations: int = 2,
        stream: str = "main",
    ):
        if (
            not isinstance(required_confirmations, int)
            or isinstance(required_confirmations, bool)
            or required_confirmations < 1
        ):
            raise ValueError("required_confirmations 必须是大于等于1的整数")
        self.camera = camera
        self.scanner = scanner or QRCodeScanner()
        self.required_confirmations = required_confirmations
        self.stream = stream
        self._candidate: Optional[str] = None
        self._confirmation_count = 0

    def reset(self) -> None:
        """清除候选文本；开始新一轮任务前可显式调用。"""
        self._candidate = None
        self._confirmation_count = 0

    def read_task_code(self) -> Optional[str]:
        """采集一帧并更新任务码确认状态。"""
        frame = self.camera.capture_array(self.stream)
        return self.update_frame(frame)

    def update_frame(self, frame) -> Optional[str]:
        """处理调用方已经采集的帧。

        前向相机同时承担巡线和二维码识别时，调用这个接口可以让两个算法复用
        同一帧，避免一次控制周期内重复向 Picamera2 请求图像。
        """
        decoded = self.scanner.decode(frame)
        if len(decoded) != 1:
            self.reset()
            return None

        candidate = decoded[0]
        if candidate == self._candidate:
            self._confirmation_count += 1
        else:
            self._candidate = candidate
            self._confirmation_count = 1

        if self._confirmation_count < self.required_confirmations:
            return None
        return candidate
