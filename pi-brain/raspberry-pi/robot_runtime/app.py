"""树莓派 systemd 服务调用的运行入口。"""

from argparse import ArgumentParser
from dataclasses import replace
import importlib
import logging
import signal
from threading import Event

from robot_simulation import build_simulated_components
from .config import DEFAULT_CONFIG_PATH, RuntimeConfig, RuntimeConfigError
from .config import load_runtime_config
from .interfaces import ComponentBundle
from .state_machine import RobotStateMachine


def _load_component_factory(path: str):
    module_name, function_name = path.split(":", 1)
    module = importlib.import_module(module_name)
    factory = getattr(module, function_name)
    if not callable(factory):
        raise TypeError(f"组件工厂不可调用：{path}")
    return factory


def build_components(config: RuntimeConfig, force_simulation: bool = False):
    if config.component_factory and not force_simulation:
        factory = _load_component_factory(config.component_factory)
        components = factory(config)
        if not isinstance(components, ComponentBundle):
            raise TypeError("组件工厂必须返回 ComponentBundle")
        return components
    return build_simulated_components(
        task_code=config.simulation_task_code,
        auto_start=config.simulation_auto_start,
        start_delay_seconds=config.simulation_start_delay_seconds,
    )


def parse_arguments():
    parser = ArgumentParser(description="智能搬运机器人底层状态机")
    parser.add_argument("--config", default=str(DEFAULT_CONFIG_PATH))
    parser.add_argument("--simulate", action="store_true", help="强制使用模拟组件")
    parser.add_argument(
        "--auto-start",
        action="store_true",
        help="仅模拟模式：自动模拟实体按钮按下",
    )
    parser.add_argument("--task-code", help="仅模拟模式：覆盖测试任务码")
    parser.add_argument(
        "--exit-on-terminal",
        action="store_true",
        help="进入完成/安全停车状态后退出；systemd 默认保持最终显示",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_arguments()
    try:
        config = load_runtime_config(args.config)
        if args.auto_start:
            config = replace(config, simulation_auto_start=True)
        if args.task_code:
            config = replace(config, simulation_task_code=args.task_code)

        logging.basicConfig(
            level=getattr(logging, config.log_level, logging.INFO),
            format="%(asctime)s %(levelname)s %(name)s: %(message)s",
        )
        components = build_components(config, force_simulation=args.simulate)
    except (RuntimeConfigError, ImportError, AttributeError, TypeError) as error:
        print(f"启动配置错误：{error}")
        return 2

    stop_event = Event()

    def request_stop(_signum, _frame):
        stop_event.set()

    signal.signal(signal.SIGINT, request_stop)
    signal.signal(signal.SIGTERM, request_stop)

    machine = RobotStateMachine(components, config)
    try:
        while not stop_event.is_set():
            machine.tick()
            if machine.is_terminal and args.exit_on_terminal:
                break
            machine.clock.sleep(config.loop_interval_seconds)
        if stop_event.is_set() and not machine.is_terminal:
            machine.safe_stop("运行服务收到停止信号")
        return 0 if machine.state.name == "COMPLETED" else 1
    finally:
        machine.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
