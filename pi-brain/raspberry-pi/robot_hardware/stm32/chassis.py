"""通过 STM32 命令控制全向底盘。"""

from .messages import Command, encode_chassis_velocity
from .serial_link import SerialLink


class Stm32ChassisController:
    """导航周期使用的非阻塞速度发送器；STM32 必须实现通信看门狗。"""

    def __init__(self, link: SerialLink, *, activity_reader=None) -> None:
        self.link = link
        self._activity_reader = activity_reader
        self._commanded_velocity = (0, 0, 0)

    def set_velocity(self, vx_mm_s: int, vy_mm_s: int, wz_mrad_s: int) -> None:
        self.link.send_command(
            Command.SET_CHASSIS_VELOCITY,
            encode_chassis_velocity(vx_mm_s, vy_mm_s, wz_mrad_s),
        )
        self._commanded_velocity = (vx_mm_s, vy_mm_s, wz_mrad_s)

    def stop(self) -> None:
        self.link.send_command(Command.STOP_ALL)
        self._commanded_velocity = (0, 0, 0)

    def is_active(self) -> bool:
        return bool(self._activity_reader and self._activity_reader())

    @property
    def commanded_velocity(self) -> tuple[int, int, int]:
        return self._commanded_velocity

    @property
    def commanded_speed_mm_s(self) -> float:
        vx, vy, _ = self._commanded_velocity
        return (vx * vx + vy * vy) ** 0.5

    @property
    def commanded_motion_active(self) -> bool:
        return any(self._commanded_velocity)
