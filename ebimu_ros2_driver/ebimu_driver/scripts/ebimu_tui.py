#!/usr/bin/env python3
"""Standalone serial menu for configuring an EBIMU before launching ROS."""

from __future__ import annotations

from dataclasses import dataclass
import time

from rich.console import Console
from rich.prompt import Prompt
import serial


DEFAULT_PORT = "/dev/imu"
BAUD_SCAN = (115200, 460800, 921600, 230400, 57600, 38400, 19200, 9600)
BAUD_COMMANDS = {
    9600: "sb1",
    19200: "sb2",
    38400: "sb3",
    57600: "sb4",
    115200: "sb5",
    230400: "sb6",
    460800: "sb7",
    921600: "sb8",
}
ASCII_100_COMMANDS = ("sor10", "soc1", "sof2", "sog1", "soa1", "som0", "sod0", "sot0", "sots1")
BIN_500_COMMANDS = ("sor10", "soc2", "sof2", "sog1", "soa1", "som0", "sod0", "sot0", "sots1", "sor2")


@dataclass
class ProbeResult:
    port: str
    baudrate: int | None = None
    response: str = ""
    stream_mode: str = "unknown"
    approx_rate_hz: float = 0.0
    valid_frames: int = 0
    preview: bytes = b""
    error: str = ""


class SerialTools:
    @staticmethod
    def open(port: str, baudrate: int, timeout: float = 0.15) -> serial.Serial:
        try:
            return serial.Serial(
                port=port,
                baudrate=baudrate,
                timeout=timeout,
                write_timeout=timeout,
                exclusive=True,
            )
        except TypeError:
            return serial.Serial(port=port, baudrate=baudrate, timeout=timeout, write_timeout=timeout)

    @staticmethod
    def wrap(command: str, raw: bool = False) -> bytes:
        if raw:
            return command.encode("ascii")
        wrapped = command.strip()
        if not wrapped.startswith("<"):
            wrapped = "<" + wrapped
        if not wrapped.endswith(">"):
            wrapped += ">"
        return wrapped.encode("ascii")

    @staticmethod
    def read_window(ser: serial.Serial, duration_sec: float) -> bytes:
        deadline = time.monotonic() + duration_sec
        data = bytearray()
        while time.monotonic() < deadline:
            chunk = ser.read(256)
            if chunk:
                data.extend(chunk)
        return bytes(data)

    @staticmethod
    def send_command(ser: serial.Serial, command: str, raw: bool = False, timeout_sec: float = 0.7) -> bytes:
        ser.reset_input_buffer()
        ser.write(SerialTools.wrap(command, raw))
        ser.flush()
        return SerialTools.read_window(ser, timeout_sec)


def response_text(data: bytes) -> str:
    start = data.find(b"<")
    end = data.find(b">", start + 1)
    if start >= 0 and end > start:
        candidate = data[start:end + 1]
        if len(candidate) <= 64 and all(32 <= byte <= 126 for byte in candidate):
            return candidate.decode("ascii")
    return ""


def count_ascii_frames(data: bytes) -> int:
    count = 0
    for raw_line in data.splitlines():
        line = raw_line.strip()
        if not line.startswith(b"*"):
            continue
        try:
            fields = [field.strip() for field in line[1:].split(b",")]
            if len(fields) >= 6:
                for field in fields:
                    float(field)
                count += 1
        except ValueError:
            continue
    return count


def count_binary_frames(data: bytes) -> int:
    best_count = 0
    for value_count in range(3, 19):
        frame_len = 2 + value_count * 2 + 2
        idx = 0
        count = 0
        while True:
            start = data.find(b"\x55\x55", idx)
            if start < 0 or start + frame_len > len(data):
                break
            frame = data[start:start + frame_len]
            checksum = sum(frame[:-2]) & 0xFFFF
            received = (frame[-2] << 8) | frame[-1]
            if checksum == received:
                count += 1
                idx = start + frame_len
            else:
                idx = start + 1
        best_count = max(best_count, count)
    return best_count


def analyze_stream(data: bytes, duration_sec: float) -> tuple[str, int, float]:
    ascii_frames = count_ascii_frames(data)
    binary_frames = count_binary_frames(data)
    if ascii_frames >= 2 and ascii_frames >= binary_frames:
        return "ascii", ascii_frames, estimate_rate(ascii_frames, duration_sec)
    if binary_frames >= 2:
        return "binary", binary_frames, estimate_rate(binary_frames, duration_sec)
    return "unknown", max(ascii_frames, binary_frames), 0.0


def estimate_rate(frame_count: int, duration_sec: float) -> float:
    if duration_sec <= 0.0:
        return 0.0
    return float(frame_count) / duration_sec


class EbimuSerialMenu:
    def __init__(self) -> None:
        self.console = Console()
        self.result = ProbeResult(port=DEFAULT_PORT)
        self.last_action = "not probed"
        self.port = DEFAULT_PORT
        self.logs: list[str] = []

    def _port(self) -> str:
        return self.port

    def _log(self, message: str) -> None:
        self.logs.append(message)
        self.logs = self.logs[-12:]

    def _render(self) -> None:
        self.console.clear()
        baud = str(self.result.baudrate) if self.result.baudrate else "unknown"
        response = self.result.response or "-"
        error = self.result.error or "-"
        launch = self._launch_hint()

        status = (
            "+---------------- EBIMU SERIAL ----------------+\n"
            f" port      : {self.result.port}\n"
            f" baudrate  : {baud}\n"
            f" stream    : {self.result.stream_mode}\n"
            f" rate      : {self.result.approx_rate_hz:7.1f} Hz\n"
            f" frames    : {self.result.valid_frames}\n"
            f" response  : {response}\n"
            f" last      : {self.last_action}\n"
            f" error     : {error[:64]}\n"
            f" launch    : {launch[:92]}\n"
            "+----------------------------------------------+"
        )
        self.console.print(status, style="cyan", markup=False)
        menu = (
            "\nBring-up\n"
            "  1. 포트/보드레이트 자동 찾기\n"
            "  2. 포트 변경\n"
            "  3. 보드레이트 변경\n"
            "  4. ASCII 100Hz 안전 설정\n"
            "  5. Binary 500Hz 권장 설정\n"
            "  6. 현재 설정 수신 검증\n"
            "  7. launch 명령 보기\n"
            "\nCalibration / Raw\n"
            "  8. Gyro calibration\n"
            "  9. Mag calibration start\n"
            " 10. Mag calibration finish\n"
            " 11. Reset\n"
            " 12. Raw command\n"
            "  0. Quit"
        )
        self.console.print(menu, style="green", markup=False)
        if self.logs:
            self.console.print("\nlog", style="yellow", markup=False)
            for line in self.logs:
                self.console.print(f"  {line}", markup=False)

    def _pause(self) -> None:
        Prompt.ask("\nEnter를 누르면 메뉴로 돌아감", default="")

    def _launch_hint(self) -> str:
        if self.result.baudrate and self.result.stream_mode in {"ascii", "binary"}:
            interval_ms = 10
            if self.result.approx_rate_hz > 300.0:
                interval_ms = 2
            elif self.result.approx_rate_hz > 0.0:
                interval_ms = max(1, round(1000.0 / self.result.approx_rate_hz))
            return (
                "ros2 launch ebimu_driver ebimu_driver.launch.py "
                f"baudrate:={self.result.baudrate} "
                f"output_mode:={self.result.stream_mode} "
                f"output_interval_ms:={interval_ms}"
            )
        if self.result.baudrate:
            return f"baud found but stream invalid at {self.result.baudrate}"
        return "probe first"

    def _probe_baudrate(self, baudrate: int, duration_sec: float = 0.8) -> ProbeResult:
        port = self._port()
        try:
            with SerialTools.open(port, baudrate) as ser:
                ser.reset_input_buffer()
                passive = SerialTools.read_window(ser, duration_sec)
                mode, valid_frames, rate = analyze_stream(passive, duration_sec)
                data = passive
                response = ""
                if valid_frames < 2:
                    command_data = SerialTools.send_command(ser, "ver", timeout_sec=0.5)
                    trailing = SerialTools.read_window(ser, duration_sec)
                    data = command_data + trailing
                    mode, valid_frames, rate = analyze_stream(data, duration_sec)
                    response = response_text(command_data)
        except serial.SerialException as exc:
            return ProbeResult(port=port, error=str(exc))
        except OSError as exc:
            return ProbeResult(port=port, error=str(exc))

        if valid_frames >= 2:
            return ProbeResult(
                port=port,
                baudrate=baudrate,
                response=response,
                stream_mode=mode,
                approx_rate_hz=rate,
                valid_frames=valid_frames,
                preview=data[:160],
            )
        if response:
            return ProbeResult(
                port=port,
                baudrate=baudrate,
                response=response,
                stream_mode="command-response",
                valid_frames=0,
                preview=data[:160],
                error="command response only; no valid stream frames",
            )
        return ProbeResult(
            port=port,
            baudrate=baudrate,
            valid_frames=valid_frames,
            preview=data[:160],
            error="no valid stream frames",
        )

    def _probe(self) -> ProbeResult:
        port = self._port()
        fallback: ProbeResult | None = None
        for baudrate in BAUD_SCAN:
            self._log(f"scan baudrate {baudrate}")
            result = self._probe_baudrate(baudrate)
            if result.error and any(
                token in result.error.lower()
                for token in ("busy", "no such file", "permission denied", "could not open port")
            ):
                return result
            if result.valid_frames >= 2:
                return result
            if result.response and fallback is None:
                fallback = result
            self._log(f"  {baudrate}: {result.error}")
        if fallback is not None:
            fallback.error = "no valid stream frames; saw command response only"
            return fallback
        return ProbeResult(port=port, error="no EBIMU response or stream at known baudrates")

    def _run_probe(self) -> None:
        self.result = self._probe()
        self.last_action = "probe"
        if self.result.preview:
            self._log(f"probe preview: {self.result.preview!r}")
        if self.result.error:
            self._log(f"probe error: {self.result.error}")

    def _with_detected_serial(self):
        if self.result.baudrate is None:
            self.result = self._probe()
            if self.result.error:
                raise serial.SerialException(self.result.error)
        return SerialTools.open(self.result.port, self.result.baudrate)

    def _send_sequence(self, commands: tuple[str, ...], label: str) -> None:
        try:
            with self._with_detected_serial() as ser:
                for command in commands:
                    data = SerialTools.send_command(ser, command, raw=command == ">", timeout_sec=0.7)
                    text = response_text(data)
                    self._log(f"{label} <{command}> -> {text or data[:80]!r}")
                    if text == "<er>":
                        raise serial.SerialException(f"device rejected <{command}>")
            self.last_action = label
            if self.result.baudrate is not None:
                self.result = self._probe_baudrate(self.result.baudrate)
        except Exception as exc:
            self.last_action = label
            self.result.error = str(exc)
            self._log(f"{label}: {exc}")

    def _set_baudrate(self, target: int) -> None:
        command = BAUD_COMMANDS[target]
        try:
            with self._with_detected_serial() as ser:
                SerialTools.send_command(ser, "sor10", timeout_sec=0.7)
                data = SerialTools.send_command(ser, command, timeout_sec=0.7)
                self._log(f"baud <{command}> -> {response_text(data) or data[:80]!r}")
            time.sleep(0.2)
            with SerialTools.open(self.result.port, target) as ser:
                ser.reset_input_buffer()
            self.result = self._probe_baudrate(target)
            self.last_action = f"baud {target}"
        except Exception as exc:
            self.last_action = f"baud {target}"
            self.result.error = str(exc)
            self._log(f"baud {target}: {exc}")

    def _prompt_port(self) -> None:
        self.port = Prompt.ask("port", default=self.port).strip() or DEFAULT_PORT
        self.result = ProbeResult(port=self.port)
        self.last_action = "port changed"
        self._log(f"port -> {self.port}")

    def _prompt_baudrate(self) -> None:
        self.console.print("\n[1] 115200    [2] 460800    [3] 921600    [4] 230400", markup=False)
        self.console.print("[5] 57600     [6] 38400     [7] 19200     [8] 9600", markup=False)
        choices = {
            "1": 115200,
            "2": 460800,
            "3": 921600,
            "4": 230400,
            "5": 57600,
            "6": 38400,
            "7": 19200,
            "8": 9600,
        }
        choice = Prompt.ask("baud 메뉴 번호", choices=[*choices.keys(), "0"], default="2")
        if choice == "0":
            return
        target = choices[choice]
        self._set_baudrate(target)

    def _prompt_raw(self) -> None:
        command = Prompt.ask("raw command 예: <ver> 또는 >").strip()
        if not command:
            return
        raw = command == ">"
        self._send_sequence((command if raw else command.strip("<>"),), f"raw {command}")

    def _verify_current(self) -> None:
        if self.result.baudrate is None:
            self.result = self._probe()
        elif self.result.stream_mode == "command-response":
            self.result = self._probe_baudrate(self.result.baudrate)
        else:
            self.result = self._probe_baudrate(self.result.baudrate)
        self.last_action = "verify current"
        if self.result.valid_frames >= 2:
            self._log(
                f"verify ok: {self.result.stream_mode}, {self.result.valid_frames} frames, "
                f"{self.result.approx_rate_hz:.1f} Hz"
            )
        else:
            self._log(f"verify failed: {self.result.error}")

    def _show_launch(self) -> None:
        self.console.print("\nlaunch command", style="yellow", markup=False)
        self.console.print(self._launch_hint(), markup=False)

    def run(self) -> None:
        while True:
            self._render()
            choice = Prompt.ask("\n번호", choices=[str(i) for i in range(0, 13)], default="1")
            if choice == "0":
                return
            if choice == "1":
                self._run_probe()
                self._pause()
                continue
            if choice == "2":
                self._prompt_port()
                continue
            if choice == "3":
                self._prompt_baudrate()
                self._pause()
                continue
            if choice == "4":
                self._send_sequence(ASCII_100_COMMANDS, "ascii 100")
                self._pause()
                continue
            if choice == "5":
                self._send_sequence(BIN_500_COMMANDS, "bin 500")
                self._pause()
                continue
            if choice == "6":
                self._verify_current()
                self._pause()
                continue
            if choice == "7":
                self._show_launch()
                self._pause()
                continue
            if choice == "8":
                self._send_sequence(("cg",), "gyro cal")
                self._pause()
                continue
            if choice == "9":
                self._send_sequence(("cmf",), "mag start")
                self._pause()
                continue
            if choice == "10":
                self._send_sequence((">",), "mag finish")
                self._pause()
                continue
            if choice == "11":
                self._send_sequence(("reset",), "reset")
                self._pause()
                continue
            if choice == "12":
                self._prompt_raw()
                self._pause()
                continue


def main() -> None:
    EbimuSerialMenu().run()


if __name__ == "__main__":
    main()
