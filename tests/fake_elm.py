"""A pretend ELM327 adapter plugged into a pretend Mercedes, served over TCP.

Used by the tests, and runnable on its own to try slkdiag without a car:

    python3 tests/fake_elm.py            # prints the port it listens on
    python3 slkdiag.py --tcp 127.0.0.1:<port>
"""

from __future__ import annotations

import socket
import threading
from dataclasses import dataclass
from typing import Callable

Handler = Callable[[bytes], "bytes | list[bytes] | None"]


@dataclass
class FakeUnit:
    response_id: int
    handler: Handler
    answers_broadcast: bool = False  # replies to the 7DF address every OBD unit hears


def isotp_frames(payload: bytes) -> list[bytes]:
    """Split a message into padded CAN frames the way a control unit sends it."""
    if len(payload) <= 7:
        frames = [bytes([len(payload)]) + payload]
    else:
        frames = [bytes([0x10 | (len(payload) >> 8), len(payload) & 0xFF]) + payload[:6]]
        rest, index = payload[6:], 1
        while rest:
            frames.append(bytes([0x20 | (index & 0x0F)]) + rest[:7])
            rest, index = rest[7:], index + 1
    return [frame.ljust(8, b"\xaa") for frame in frames]


class FakeElm(threading.Thread):
    def __init__(
        self,
        units: dict[int, FakeUnit],
        *,
        receive_address: bool = True,
        filter_mask: bool = True,
        flow_control: bool = True,
        background: tuple[str, ...] = (),
        version: str = "ELM327 v1.5",
    ):
        super().__init__(daemon=True)
        self.units = units
        self.supports_receive_address = receive_address
        self.supports_filter_mask = filter_mask
        self.supports_flow_control = flow_control
        self.background = background
        self.version = version
        self.sent: list[tuple[int, bytes]] = []  # every request put on the bus
        self._server = socket.create_server(("127.0.0.1", 0))
        self.port = self._server.getsockname()[1]
        self._reset()

    def _reset(self) -> None:
        self.echo = True
        self.spaces = True
        self.headers = False
        self.header = 0x7DF
        self.filter: int | None = None
        self.mask: int | None = None
        self.flow_mode = 0
        self.flow_header: int | None = None
        self.flow_data: str | None = None
        self.monitoring = False

    def run(self) -> None:
        conn, _ = self._server.accept()
        self._server.close()
        buffer = bytearray()
        with conn:
            while True:
                try:
                    chunk = conn.recv(1024)
                except OSError:
                    return
                if not chunk:
                    return
                buffer += chunk
                while buffer:
                    if self.monitoring:  # any one character stops monitoring
                        del buffer[0]
                        self.monitoring = False
                        conn.sendall(b"STOPPED\r\r>")
                        continue
                    end = buffer.find(b"\r")
                    if end < 0:
                        break
                    line = buffer[:end].decode("ascii")
                    del buffer[: end + 1]
                    conn.sendall(self._handle(line))

    def _handle(self, line: str) -> bytes:
        command = line.replace(" ", "").upper()
        echo = (line + "\r") if self.echo else ""
        if command.startswith("AT"):
            lines = self._at(command[2:])
        else:
            lines = self._data(command)
        if self.monitoring:
            return (echo + "".join(text + "\r" for text in lines)).encode("ascii")
        return (echo + "\r".join(lines) + "\r\r>").encode("ascii")

    def _at(self, command: str) -> list[str]:
        ok, unknown = ["OK"], ["?"]
        if command == "Z":
            self._reset()
            return ["", self.version]
        if command == "I":
            return [self.version]
        if command == "@1":
            return ["OBDII to RS232 Interpreter"]
        if command == "RV":
            return ["12.4V"]
        if command == "D":
            self.header, self.filter, self.mask, self.flow_mode = 0x7DF, None, None, 0
            return ok
        if command in ("E0", "E1"):
            self.echo = command == "E1"
            return ok
        if command in ("S0", "S1"):
            self.spaces = command == "S1"
            return ok
        if command in ("H0", "H1"):
            self.headers = command == "H1"
            return ok
        if command in ("L0", "L1", "CAF0", "CAF1", "AT0", "AT1", "AT2", "SP6"):
            return ok
        if command.startswith("ST") and len(command) == 4:
            return ok
        if command.startswith("SH") and len(command) == 5:
            self.header = int(command[2:], 16)
            return ok
        if command in ("CRA", "AR"):
            self.filter = self.mask = None
            return ok
        if command.startswith("CRA"):
            if not self.supports_receive_address:
                return unknown
            self.filter, self.mask = int(command[3:], 16), 0x7FF
            return ok
        if command.startswith("CF") or command.startswith("CM"):
            if not self.supports_filter_mask:
                return unknown
            if command.startswith("CF"):
                self.filter = int(command[2:], 16)
            else:
                self.mask = int(command[2:], 16)
            return ok
        if command.startswith("FCS"):
            if not self.supports_flow_control:
                return unknown
            if command.startswith("FCSH"):
                self.flow_header = int(command[4:], 16)
            elif command.startswith("FCSD"):
                self.flow_data = command[4:]
            elif command == "FCSM1":
                if self.flow_header is None or self.flow_data is None:
                    return unknown
                self.flow_mode = 1
            else:
                self.flow_mode = int(command[4:])
            return ok
        if command == "MA":
            self.monitoring = True
            return list(self.background)
        return unknown

    def _passes(self, sender: int) -> bool:
        if self.filter is None or self.mask is None:
            return 0x7E8 <= sender <= 0x7EF
        return (sender & self.mask) == (self.filter & self.mask)

    def _format(self, sender: int, frame: bytes) -> str:
        body = [f"{byte:02X}" for byte in frame]
        parts = ([f"{sender:03X}"] if self.headers else []) + body
        return (" " if self.spaces else "").join(parts)

    def _data(self, command: str) -> list[str]:
        try:
            payload = bytes.fromhex(command)
        except ValueError:
            return ["?"]
        self.sent.append((self.header, payload))
        if self.header == 0x7DF:
            addressed = [unit for unit in self.units.values() if unit.answers_broadcast]
        else:
            addressed = [self.units[self.header]] if self.header in self.units else []
        lines: list[str] = []
        for unit in addressed:
            answer = unit.handler(payload)
            if answer is None:
                continue
            request_id = next(key for key, value in self.units.items() if value is unit)
            for message in answer if isinstance(answer, list) else [answer]:
                frames = isotp_frames(message)
                if len(frames) > 1 and not self._flow_control_reaches(request_id, unit):
                    frames = frames[:1]  # the unit waits forever for our go-ahead
                lines += [
                    self._format(unit.response_id, frame)
                    for frame in frames
                    if self._passes(unit.response_id)
                ]
        return lines or ["NO DATA"]

    def _flow_control_reaches(self, request_id: int, unit: FakeUnit) -> bool:
        if self.flow_mode == 1:
            return self.flow_header == request_id
        return unit.response_id == request_id + 8  # the adapter's built-in assumption


# --- A pretend SLK ----------------------------------------------------------


def uds_unit(part_number: str, faults: bytes = b"") -> Handler:
    def handler(payload: bytes):
        if payload == b"\x3e\x00":
            return b"\x7e\x00"
        if payload[:2] == b"\x19\x02":
            return b"\x59\x02\xff" + faults
        if payload == b"\x22\xf1\x11":
            return b"\x62\xf1\x11" + part_number.encode("ascii")
        if payload[:1] == b"\x22":
            return b"\x7f\x22\x31"
        return bytes([0x7F, payload[0], 0x11])

    return handler


def engine_unit(payload: bytes):
    if payload == b"\x01\x00":
        return b"\x41\x00\xbe\x3f\xa8\x13"
    return uds_unit("2769001234")(payload)


def kwp_unit(payload: bytes):
    """An older unit: silent on tester-present, refuses the UDS fault request."""
    if payload == b"\x18\x02\xff\x00":
        return b"\x58\x01\x95\x1d\x60"
    if payload == b"\x1a\x86":
        return b"\x5a\x86" + b"KW2C3PE"
    if payload == b"\x3e\x00":
        return None
    return bytes([0x7F, payload[0], 0x11])


CLIMATE_FAULTS = bytes.fromhex("D0080009" "912F8709" "91870708")


def pretend_slk() -> dict[int, FakeUnit]:
    return {
        0x7E0: FakeUnit(0x7E8, engine_unit, answers_broadcast=True),
        0x612: FakeUnit(0x482, uds_unit("1729051700")),
        0x672: FakeUnit(0x48E, uds_unit("1729003304", CLIMATE_FAULTS)),
        0x6C2: FakeUnit(0x498, kwp_unit),
        0x743: FakeUnit(0x4E8, uds_unit("1729012200")),  # outside the usual scheme
    }


if __name__ == "__main__":
    adapter = FakeElm(pretend_slk())
    adapter.start()
    print(f"Pretend adapter listening on 127.0.0.1:{adapter.port}", flush=True)
    adapter.join()
