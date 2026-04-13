"""GPIO adapter layer — all gpiod calls are isolated here."""

from __future__ import annotations


class GpioAdapter:
    """Wraps gpiod v2 API for DIO read/write on a single chip."""

    def __init__(
        self,
        chip_path: str,
        do_lines: list[int],
        di_lines: list[int],
        consumer: str = "dio_driver",
    ):
        self._do_lines = list(do_lines)
        self._di_lines = list(di_lines)
        self._connected = False
        self._do_request = None
        self._di_request = None
        self._error = ""

        try:
            import gpiod
            from gpiod.line import Direction

            do_config = {
                tuple(self._do_lines): gpiod.LineSettings(direction=Direction.OUTPUT)
            }
            di_config = {
                tuple(self._di_lines): gpiod.LineSettings(direction=Direction.INPUT)
            }

            self._do_request = gpiod.request_lines(
                chip_path, consumer=consumer, config=do_config
            )
            self._di_request = gpiod.request_lines(
                chip_path, consumer=consumer, config=di_config
            )
            self._connected = True

        except PermissionError:
            self._error = (
                f"Permission denied for {chip_path}. "
                "Run: sudo usermod -aG gpio $USER && reboot"
            )
        except OSError as e:
            self._error = f"Failed to open {chip_path}: {e}"
        except Exception as e:
            self._error = f"GPIO init error: {e}"

    @property
    def connected(self) -> bool:
        return self._connected

    @property
    def error(self) -> str:
        return self._error

    def read_inputs(self) -> list[bool]:
        if not self._connected or self._di_request is None:
            return [False] * len(self._di_lines)
        from gpiod.line import Value

        values = self._di_request.get_values(self._di_lines)
        return [v == Value.ACTIVE for v in values]

    def read_outputs(self) -> list[bool]:
        if not self._connected or self._do_request is None:
            return [False] * len(self._do_lines)
        from gpiod.line import Value

        values = self._do_request.get_values(self._do_lines)
        return [v == Value.ACTIVE for v in values]

    def set_output(self, channel: int, value: bool) -> None:
        if not self._connected or self._do_request is None:
            raise OSError("GPIO not connected")
        if channel < 0 or channel >= len(self._do_lines):
            raise ValueError(
                f"Channel {channel} out of range [0, {len(self._do_lines) - 1}]"
            )
        from gpiod.line import Value

        line = self._do_lines[channel]
        self._do_request.set_value(line, Value.ACTIVE if value else Value.INACTIVE)

    def close(self) -> None:
        if self._do_request is not None:
            self._do_request.release()
            self._do_request = None
        if self._di_request is not None:
            self._di_request.release()
            self._di_request = None
        self._connected = False


class FakeGpioAdapter:
    """In-memory stub for testing/simulation without hardware."""

    def __init__(self, do_lines: list[int], di_lines: list[int]):
        self._do_state = [False] * len(do_lines)
        self._di_state = [False] * len(di_lines)

    @property
    def connected(self) -> bool:
        return True

    @property
    def error(self) -> str:
        return ""

    def read_inputs(self) -> list[bool]:
        return list(self._di_state)

    def read_outputs(self) -> list[bool]:
        return list(self._do_state)

    def set_output(self, channel: int, value: bool) -> None:
        if channel < 0 or channel >= len(self._do_state):
            raise ValueError(
                f"Channel {channel} out of range [0, {len(self._do_state) - 1}]"
            )
        self._do_state[channel] = value

    def close(self) -> None:
        pass
