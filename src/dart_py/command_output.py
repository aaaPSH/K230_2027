"""制导指令输出接口。"""


class OverloadCommandOutput:
    """最终过载指令的输出接口。"""

    def __init__(self, config=None):
        self.config = config or {}

    def send_overload(self, command):
        """向下位机发送一个指令字典。默认实现为空操作。"""
        return True

    def deinit(self):
        """释放输出设备资源。"""
        return None


class ConsoleCommandOutput(OverloadCommandOutput):
    """可选的调试输出，将指令字典打印到控制台。"""

    def send_overload(self, command):
        print(
            "cmd detected={} pitch_g={:.3f} yaw_g={:.3f} fps={:.1f}".format(
                command.get("detected", False),
                command.get("pitch_overload_g", 0.0),
                command.get("yaw_overload_g", 0.0),
                command.get("fps", 0.0),
            )
        )
        return True


def make_lower_computer_interface(config):
    if config.get("debug_print", False):
        return ConsoleCommandOutput(config)
    return OverloadCommandOutput(config)
