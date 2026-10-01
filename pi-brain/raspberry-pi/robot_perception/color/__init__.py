"""颜色识别算法与参数加载。

检测器依赖OpenCV，应从 ``robot_perception.color.detector`` 显式导入；这里不
提前导入它，以便开发机在没有OpenCV时仍能校验配置文件。
"""

from .config import ConfigError, load_config

__all__ = ["ConfigError", "load_config"]
