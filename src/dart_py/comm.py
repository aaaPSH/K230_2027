"""Hardware communication interfaces.

These classes are intentionally protocol-neutral. Replace them with UART/CAN/SPI
implementations once the lower-computer and IMU wire formats are fixed.
"""


class LowerComputerInterface:
    """Output interface for final overload commands."""

    def __init__(self, config=None):
        self.config = config or {}

    def send_overload(self, command):
        """Send one command dictionary to the lower computer.

        Default implementation is a no-op and returns True so the guidance loop
        can run on a bare K230/IDE setup before the physical link is wired.
        """
        return True


class GyroInterface:
    """Input interface for roll/pitch/gyro body-frame data."""

    def __init__(self, config=None):
        self.config = config or {}

    def read(self):
        """Return None or {'roll_rad': float, 'gyro_b': list}.

        gyro_b must be [roll_rate, pitch_rate, yaw_rate] in body frame, rad/s.
        """
        return None


class ConsoleLowerComputerInterface(LowerComputerInterface):
    """Optional debug sender that prints the command dictionary."""

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
        return ConsoleLowerComputerInterface(config)
    return LowerComputerInterface(config)


def make_gyro_interface(config):
    return GyroInterface(config)
