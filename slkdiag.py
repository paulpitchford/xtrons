#!/usr/bin/env python3
"""Read-only fault-code reader for Mercedes control units through an ELM327 adapter.

Generic OBD apps only reach the engine. This talks to the other control units
(climate control, ignition switch, body modules) on Mercedes' own diagnostic
addresses, lists which ones answer, and reads their stored fault codes.

It never writes to the car: only "tester present", "read fault codes" and
"read identification" requests can be sent (see READ_ONLY_SERVICES).
"""

from __future__ import annotations

import argparse
import errno
import json
import os
import re
import select
import socket
import sys
import time
from dataclasses import asdict, dataclass, field
from datetime import datetime
from pathlib import Path

LOG_DIR = Path(__file__).resolve().parent / "logs"

# The only diagnostic services this tool may send. Anything else is refused
# before it reaches the adapter, so a bug here cannot clear codes or change coding.
READ_ONLY_SERVICES = {
    0x01: "OBD-II live data",
    0x18: "KWP2000 read fault codes",
    0x19: "UDS read fault codes",
    0x1A: "KWP2000 read identification",
    0x22: "UDS read data by identifier",
    0x3E: "tester present",
}

NEGATIVE_RESPONSES = {
    0x10: "general reject",
    0x11: "service not supported",
    0x12: "sub-function not supported",
    0x13: "wrong message length",
    0x22: "conditions not correct",
    0x31: "request out of range",
    0x33: "security access denied",
    0x78: "response pending",
    0x7E: "sub-function not supported in this session",
    0x7F: "service not supported in this session",
}

# Identification values worth asking a UDS unit for. F111 is Mercedes' own
# hardware part number; the other two are generic UDS identifiers.
UDS_IDENTIFIERS = {
    0xF111: "hardware part number",
    0xF187: "spare part number",
    0xF197: "system name",
}

# Meanings reported on owner forums and in one service bulletin for related
# Mercedes models. Keyed by the first two bytes; the third byte is the failure type.
CODE_HINTS = {
    "D008": "LIN bus 1 faulty",
    "912F": "front blower regulator malfunction",
    "9187": "air flap actuator fault",
    "912B": "air flap actuator fault",
    "90F7": "air flap actuator fault",
    "90A9": "air flap actuator fault",
    "90A2": "air flap actuator fault",
    "951D": "blower regulator LIN communication fault",
    "955D": "LIN bus: voltage or data wire interrupted",
}

DEFAULT_STATUS_MASK = 0x0D  # test failed now | pending | confirmed
SCAN_TIMEOUT_MS = 200
SWEEP_TIMEOUT_MS = 100
SWEEP_FLOOD_SECONDS = 3.0  # a sweep reply still arriving after this long is a flood
READ_TIMEOUT_MS = 400
SLOW_UNIT_TIMEOUT_MS = 1020  # the longest wait the adapter supports

FRAME_SPACED = re.compile(r"^([0-9A-F]{3,4})((?: [0-9A-F]{2}){1,8})$")
FRAME_COMPACT = re.compile(r"^([0-9A-F]{3})((?:[0-9A-F]{2}){1,8})$")


class ElmError(Exception):
    """The adapter or the car did something that stops the session."""


class ReadOnlyViolation(ElmError):
    """A request outside READ_ONLY_SERVICES was about to be sent."""


def ensure_read_only(payload: bytes) -> None:
    if not payload or payload[0] not in READ_ONLY_SERVICES:
        raise ReadOnlyViolation(f"Refusing to send {payload.hex(' ')}: not a read-only request.")
    if len(payload) > 7:
        raise ReadOnlyViolation(f"Refusing to send {payload.hex(' ')}: longer than one frame.")


# --- Bus addressing ---------------------------------------------------------


def standard_targets() -> list[tuple[int, int]]:
    """The emissions-related units every car exposes: request 7E0-7E7, reply 7E8-7EF."""
    return [(0x7E0 + n, 0x7E8 + n) for n in range(8)]


def mercedes_targets() -> list[tuple[int, int]]:
    """Mercedes' own scheme: request 0x600 + 16n + 2, reply 0x480 + 2n."""
    return [(0x600 + (n << 4) + 2, 0x480 + (n << 1)) for n in range(32)]


# --- Message reassembly -----------------------------------------------------


class Message:
    """One diagnostic message rebuilt from CAN frames; may be cut short."""

    def __init__(self, data: bytes, expected: int):
        self.data = data
        self.expected = expected

    @property
    def complete(self) -> bool:
        return len(self.data) >= self.expected


def reassemble(frames: list[bytes]) -> list[Message]:
    """Rebuild ISO-TP messages from the raw frames of one sender."""
    messages: list[Message] = []
    current: Message | None = None
    for frame in frames:
        if not frame:
            continue
        kind = frame[0] >> 4
        if kind == 0:
            length = frame[0] & 0x0F
            if length:
                messages.append(Message(bytes(frame[1 : 1 + length]), length))
            current = None
        elif kind == 1 and len(frame) >= 2:
            total = ((frame[0] & 0x0F) << 8) | frame[1]
            current = Message(bytes(frame[2:8]), total)
            messages.append(current)
        elif kind == 2 and current is not None:
            current.data = (current.data + bytes(frame[1:8]))[: current.expected]
    return messages


def parse_frame_line(line: str) -> tuple[int, bytes] | None:
    """Turn '482 02 7E 00' (or the no-spaces form) into (0x482, b'\\x02\\x7e\\x00')."""
    text = line.split("<")[0].strip().upper()
    match = FRAME_SPACED.match(text)
    if match:
        return int(match.group(1), 16), bytes.fromhex(match.group(2).replace(" ", ""))
    match = FRAME_COMPACT.match(text)
    if match:
        return int(match.group(1), 16), bytes.fromhex(match.group(2))
    return None


@dataclass
class Reply:
    frames: list[tuple[int, bytes]]
    notes: list[str]  # adapter messages such as NO DATA or CAN ERROR
    cut_short: bool = False  # the adapter was still receiving when we stopped waiting

    @classmethod
    def parse(cls, lines: list[str]) -> "Reply":
        frames, notes = [], []
        for line in lines:
            frame = parse_frame_line(line)
            if frame:
                frames.append(frame)
            else:
                notes.append(line)
        return cls(frames, notes)

    def from_id(self, can_id: int) -> list[bytes]:
        return [data for sender, data in self.frames if sender == can_id]

    def ids(self) -> list[int]:
        return list(dict.fromkeys(sender for sender, _ in self.frames))

    def loudest(self) -> int | None:
        """The address that sent the most frames, or None if nothing was heard."""
        counts: dict[int, int] = {}
        for sender, _ in self.frames:
            counts[sender] = counts.get(sender, 0) + 1
        return max(counts, key=counts.__getitem__) if counts else None


@dataclass
class Answer:
    kind: str  # "positive", "negative", "none" or "unexpected"
    data: bytes = b""
    expected: int = 0
    nrc: int | None = None
    notes: list[str] = field(default_factory=list)

    @property
    def complete(self) -> bool:
        return len(self.data) >= self.expected

    def describe(self) -> str:
        if self.kind == "negative":
            reason = NEGATIVE_RESPONSES.get(self.nrc, "unknown reason")
            return f"refused ({reason}, code {self.nrc:02X})"
        if self.kind == "none":
            return "no answer" + (f" ({', '.join(self.notes)})" if self.notes else "")
        if self.kind == "unexpected":
            return f"unexpected reply {self.data.hex(' ')}"
        return "ok"


# --- Adapter ----------------------------------------------------------------


@dataclass
class Capabilities:
    receive_address: bool = False  # ATCRA: listen on one chosen address
    filter_mask: bool = False  # ATCF/ATCM: listen on an address or a range
    flow_control: bool = False  # ATFCSH/SD/SM: needed for replies longer than one frame
    timeout: bool = False  # ATST
    raw_monitor: bool = False  # ATCAF0, for listening to background traffic

    @property
    def can_listen_anywhere(self) -> bool:
        return self.receive_address or self.filter_mask


def clean_lines(raw: bytes, echo: str) -> list[str]:
    text = raw.decode("ascii", errors="replace").replace("\x00", "").replace(">", "")
    lines = [part.strip() for part in re.split(r"[\r\n]+", text)]
    return [line for line in lines if line and line != echo]


class Elm:
    """An ELM327 adapter on an already-connected socket."""

    def __init__(self, sock: socket.socket, log=None):
        self.sock = sock
        self.log = log
        self.caps = Capabilities()
        self._header: int | None = None
        self._flow_header: int | None = None
        self._listening: object = None
        self._timeout_ms: int | None = None

    def _note(self, direction: str, text: str) -> None:
        if self.log:
            self.log.write(f"{time.strftime('%H:%M:%S')} {direction} {text}\n")
            self.log.flush()

    def _read_until_prompt(self, timeout: float) -> tuple[bytes, bool]:
        deadline = time.monotonic() + timeout
        buffer = bytearray()
        while b">" not in buffer:
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                return bytes(buffer), False
            self.sock.settimeout(remaining)
            try:
                chunk = self.sock.recv(4096)
            except (socket.timeout, TimeoutError):
                return bytes(buffer), False
            if not chunk:
                raise ElmError("The adapter closed the connection.")
            buffer += chunk
        return bytes(buffer), True

    def exchange(self, text: str, timeout: float) -> tuple[list[str], bool]:
        """Send one line and collect the reply; the flag says whether the prompt came back."""
        self._note(">>", text)
        self.sock.sendall(text.encode("ascii") + b"\r")
        raw, prompted = self._read_until_prompt(timeout)
        lines = clean_lines(raw, echo=text)
        for line in lines:
            self._note("<<", line)
        return lines, prompted

    def command(self, text: str, timeout: float = 6.0, tolerate_timeout: bool = False) -> list[str]:
        lines, prompted = self.exchange(text, timeout)
        if not prompted and not tolerate_timeout:
            raise ElmError(f"The adapter did not reply to {text} within {timeout:.0f}s.")
        return lines

    def interrupt(self) -> None:
        """Stop a reception the adapter is still busy with, and drain what it sent meanwhile."""
        self._note(">>", "(any character, to stop receiving)")
        self.sock.sendall(b" ")
        tail, _ = self._read_until_prompt(3.0)
        lines = clean_lines(tail, echo="")
        frames = sum(1 for line in lines if parse_frame_line(line))
        if frames:
            self._note("<<", f"({frames} more frames discarded)")
        for line in lines:
            if not parse_frame_line(line):
                self._note("<<", line)

    def at(self, command: str) -> list[str]:
        return self.command("AT" + command)

    def at_ok(self, command: str) -> bool:
        return "OK" in self.at(command)

    def _must(self, command: str) -> None:
        if not self.at_ok(command):
            raise ElmError(f"The adapter rejected AT{command}.")

    def reset(self) -> None:
        # Clones can stay silent on ATZ and may echo the first command after it. A
        # Bluetooth serial port can also drop the first bytes while its link comes up,
        # so an unanswered reset is sent once more.
        for _ in range(2):
            if self.command("ATZ", timeout=4.0, tolerate_timeout=True):
                break
        self.command("ATE0", tolerate_timeout=True)
        for command in ("E0", "L0", "S1", "H1"):
            self._must(command)
        self._header = self._flow_header = self._listening = self._timeout_ms = None

    def describe(self) -> dict[str, str]:
        def first(command: str) -> str:
            lines = self.at(command)
            return lines[0] if lines else ""

        return {"version": first("I"), "device": first("@1"), "battery": first("RV")}

    def use_can_500k(self) -> None:
        self._must("SP6")  # ISO 15765-4, 11-bit addresses, 500 kbit/s

    def detect_capabilities(self) -> Capabilities:
        """Ask the adapter which of the commands we rely on it understands."""
        if not self.at_ok("SH7E0"):
            raise ElmError("This adapter cannot address a specific control unit (ATSH).")
        caps = self.caps
        caps.receive_address = self.at_ok("CRA7E8")
        caps.filter_mask = self.at_ok("CF7E8") and self.at_ok("CM7FF")
        caps.flow_control = (
            self.at_ok("FCSH7E0") and self.at_ok("FCSD300000") and self.at_ok("FCSM1")
        )
        caps.timeout = self.at_ok("ST32")
        caps.raw_monitor = self.at_ok("CAF0") and self.at_ok("CAF1")
        self.at_ok("AT0")  # fixed timeouts: adaptive timing can cut slow units short
        self._header = self._flow_header = self._listening = self._timeout_ms = None
        return caps

    def set_timeout(self, milliseconds: int) -> None:
        if not self.caps.timeout or milliseconds == self._timeout_ms:
            return
        self._must(f"ST{max(1, min(255, milliseconds // 4)):02X}")
        self._timeout_ms = milliseconds

    def set_request(self, request_id: int) -> None:
        if self._header != request_id:
            self._must(f"SH{request_id:03X}")
            self._header = request_id
        if self.caps.flow_control and self._flow_header != request_id:
            self._must(f"FCSH{request_id:03X}")
            self._flow_header = request_id

    def listen_to(self, response_id: int) -> None:
        if self._listening == response_id:
            return
        if self.caps.receive_address:
            self._must(f"CRA{response_id:03X}")
        elif self.caps.filter_mask:
            self._must(f"CF{response_id:03X}")
            self._must("CM7FF")
        elif not 0x7E8 <= response_id <= 0x7EF:
            raise ElmError("This adapter can only listen on the standard engine addresses.")
        self._listening = response_id

    def listen_to_range(self, pattern: int, mask: int) -> None:
        if self._listening == (pattern, mask):
            return
        self._must(f"CF{pattern:03X}")
        self._must(f"CM{mask:03X}")
        self._listening = (pattern, mask)

    def select(self, request_id: int, response_id: int) -> None:
        self.set_request(request_id)
        self.listen_to(response_id)

    def request(self, payload: bytes, timeout: float = 6.0, tolerate_timeout: bool = False) -> Reply:
        """Send a request to the car. A reply that never reaches the prompt raises, unless
        tolerated, in which case it comes back marked cut_short with whatever was heard."""
        ensure_read_only(payload)
        text = payload.hex().upper()
        lines, prompted = self.exchange(text, timeout)
        if not prompted and not tolerate_timeout:
            raise ElmError(f"The adapter did not reply to {text} within {timeout:.0f}s.")
        reply = Reply.parse(lines)
        reply.cut_short = not prompted
        return reply

    def ask(self, response_id: int, payload: bytes) -> Answer:
        """Send one request to the selected unit and classify its answer."""
        normal_timeout = self._timeout_ms
        reply = self.request(payload)
        final, pending = self._split_pending(reply, response_id)
        if pending and not final:
            # The unit asked for more time; wait as long as the adapter can, then ask again.
            self.set_timeout(SLOW_UNIT_TIMEOUT_MS)
            reply = self.request(payload)
            final, pending = self._split_pending(reply, response_id)
            if normal_timeout is not None:
                self.set_timeout(normal_timeout)
        if not final:
            return Answer("none", notes=reply.notes)
        message = final[0]
        data = message.data
        if data[:1] == b"\x7f" and len(data) >= 3:
            return Answer("negative", data, message.expected, nrc=data[2])
        if data and data[0] == payload[0] + 0x40:
            return Answer("positive", data, message.expected)
        return Answer("unexpected", data, message.expected)

    @staticmethod
    def _split_pending(reply: Reply, response_id: int) -> tuple[list[Message], list[Message]]:
        final, pending = [], []
        for message in reassemble(reply.from_id(response_id)):
            is_pending = (
                message.data[:1] == b"\x7f" and len(message.data) >= 3 and message.data[2] == 0x78
            )
            (pending if is_pending else final).append(message)
        return final, pending

    def monitor(self, seconds: float) -> tuple[set[int], bool]:
        """Listen without transmitting. Returns the addresses heard and whether traffic overflowed."""
        self.at_ok("CAF0")
        self._note(">>", "ATMA")
        self.sock.sendall(b"ATMA\r")
        deadline = time.monotonic() + seconds
        buffer = bytearray()
        while b">" not in buffer:  # a prompt here means the adapter gave up by itself
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                break
            self.sock.settimeout(remaining)
            try:
                chunk = self.sock.recv(4096)
            except (socket.timeout, TimeoutError):
                break
            if not chunk:
                raise ElmError("The adapter closed the connection.")
            buffer += chunk
        if b">" not in buffer:
            self.sock.sendall(b" ")  # any character stops monitoring
            tail, _ = self._read_until_prompt(3.0)  # some clones never send the prompt
            buffer += tail
        lines = clean_lines(bytes(buffer), echo="ATMA")
        for line in lines:
            self._note("<<", line)
        heard = {frame[0] for frame in map(parse_frame_line, lines) if frame}
        overflowed = any("BUFFER FULL" in line for line in lines)
        self.command("ATCAF1", tolerate_timeout=True)  # also absorbs a late STOPPED
        if self.caps.raw_monitor:
            self._must("CAF1")
        return heard, overflowed

    def restore_defaults(self) -> None:
        try:
            self.command("ATD", timeout=2.0, tolerate_timeout=True)
        except (ElmError, OSError):
            pass


# --- Fault codes ------------------------------------------------------------


@dataclass
class FaultCode:
    code: str  # raw hex as Mercedes tools show it, e.g. "912F87"
    label: str  # the same code in letter form, e.g. "B112F-87"
    state: str  # "current", "stored", ...
    status: int | None
    hint: str = ""


def letter_form(code: bytes) -> str:
    first, second = code[0], code[1]
    text = f"{'PCBU'[first >> 6]}{(first >> 4) & 3}{first & 0x0F:X}{second:02X}"
    return f"{text}-{code[2]:02X}" if len(code) > 2 else text


def uds_state(status: int | None) -> str:
    if status is None:
        return "status cut off"
    parts = [
        name
        for bit, name in ((0x01, "current"), (0x08, "stored"), (0x04, "pending"))
        if status & bit
    ]
    return ", ".join(parts) or f"status {status:02X}"


def kwp_state(status: int) -> str:
    storage = {0x60: "current", 0x20: "stored", 0x40: "intermittent"}.get(status & 0x60)
    return storage or f"status {status:02X}"


def make_fault(code: bytes, status: int | None, state: str) -> FaultCode:
    raw = code.hex().upper()
    return FaultCode(raw, letter_form(code), state, status, CODE_HINTS.get(raw[:4], ""))


def parse_uds_faults(answer: Answer) -> tuple[list[FaultCode], int]:
    """Decode a 59 02 reply. Returns the codes received and how many the unit holds."""
    body = answer.data[3:]
    held = max(0, (answer.expected - 3) // 4)
    whole = len(body) // 4 * 4
    faults = [
        make_fault(body[i : i + 3], body[i + 3], uds_state(body[i + 3])) for i in range(0, whole, 4)
    ]
    if len(body) - whole == 3:  # the reply was cut off just before this code's status byte
        faults.append(make_fault(body[whole:], None, uds_state(None)))
    return faults, held


def parse_kwp_faults(answer: Answer) -> tuple[list[FaultCode], int]:
    """Decode a 58 reply: a count, then two code bytes and a status byte each."""
    held = answer.data[1] if len(answer.data) > 1 else 0
    body = answer.data[2:]
    faults = [
        make_fault(body[i : i + 2], body[i + 2], kwp_state(body[i + 2]))
        for i in range(0, len(body) // 3 * 3, 3)
    ]
    return faults, held


def readable(value: bytes) -> str:
    text = value.rstrip(b"\x00 ")
    if text and all(32 <= byte < 127 for byte in text):
        return text.decode("ascii")
    return value.hex(" ").upper()


# --- Session ----------------------------------------------------------------


@dataclass
class Unit:
    request_id: int
    response_id: int
    found_by: str  # "standard", "mercedes" or "sweep"
    probe: str  # the request that got an answer, and the answer
    protocol: str = "unknown"
    identity: dict[str, str] = field(default_factory=dict)
    faults: list[FaultCode] = field(default_factory=list)
    faults_held: int | None = None
    fault_note: str = ""

    @property
    def address(self) -> str:
        return f"{self.request_id:03X}/{self.response_id:03X}"


@dataclass
class Options:
    wide: bool = False
    mask: int = DEFAULT_STATUS_MASK
    listen_seconds: float = 2.0
    probe_only: bool = False


def probe(elm: Elm, request_id: int, response_id: int, mask: int) -> str | None:
    """Return a description of the first read-only request this address answers, if any."""
    elm.select(request_id, response_id)
    for payload in (b"\x3e\x00", bytes([0x19, 0x02, mask]), b"\x18\x02\xff\x00"):
        reply = elm.request(payload)
        frames = reply.from_id(response_id)
        if frames:
            return f"{payload.hex(' ').upper()} -> {frames[0].hex(' ').upper()}"
        if any("CAN ERROR" in note or "UNABLE TO CONNECT" in note for note in reply.notes):
            raise ElmError(
                "The adapter cannot talk on the car's network (CAN ERROR). Is the ignition on?"
            )
    return None


def sweep(elm: Elm, skip: set[int], ignore_senders: set[int], mask: int, say) -> list[Unit]:
    """Try every address from 600 to 7FF that the known schemes did not cover."""
    if not elm.caps.filter_mask:
        say("  The adapter cannot listen on a range of addresses, so the wide sweep is skipped.")
        return []
    found: list[Unit] = []
    elm.set_timeout(SWEEP_TIMEOUT_MS)
    hearing = (0x400, 0x400)  # everything from 400 to 7FF

    def ask_everyone(request_id: int) -> Reply | None:
        """Send tester-present while hearing the whole range. A unit that transmits without
        pause keeps the adapter receiving forever; it is then ignored and the sweep goes on
        hearing only the half of the range it is not in. Returns None once that fails too."""
        nonlocal hearing
        elm.set_request(request_id)
        elm.listen_to_range(*hearing)
        reply = elm.request(b"\x3e\x00", timeout=SWEEP_FLOOD_SECONDS, tolerate_timeout=True)
        if not reply.cut_short:
            return reply
        elm.interrupt()
        loud = reply.loudest()
        if loud is None or hearing[1] != 0x400:
            say("  The sweep was stopped: the adapter kept being flooded with traffic.")
            return None
        hearing = (0x600 if loud < 0x600 else 0x400, 0x600)
        low, high = hearing[0], hearing[0] + 0x1FF
        say(f"  {loud:03X} transmits without pause; ignoring it and hearing only {low:03X}-{high:03X}")
        ignore_senders.add(loud)
        elm.listen_to_range(*hearing)
        reply = elm.request(b"\x3e\x00", timeout=SWEEP_FLOOD_SECONDS, tolerate_timeout=True)
        if reply.cut_short:
            elm.interrupt()
            say("  The sweep was stopped: the adapter kept being flooded with traffic.")
            return None
        return reply

    for request_id in range(0x600, 0x800):
        if request_id in skip:
            continue
        reply = ask_everyone(request_id)
        if reply is None:
            break
        senders = [s for s in reply.ids() if s not in ignore_senders and s != request_id]
        if len(senders) == 1:
            answer = probe(elm, request_id, senders[0], mask)
            if answer:
                unit = Unit(request_id, senders[0], "sweep", answer)
                found.append(unit)
                say(f"  {unit.address} answers")
        elif senders:
            listed = ", ".join(f"{s:03X}" for s in senders)
            say(f"  {request_id:03X} got answers from several units ({listed}); skipped")
    return found


def read_unit(elm: Elm, unit: Unit, mask: int) -> None:
    """Fill in a unit's fault codes and identification."""
    elm.select(unit.request_id, unit.response_id)
    answer = elm.ask(unit.response_id, bytes([0x19, 0x02, mask]))
    if answer.kind == "positive":
        unit.protocol = "UDS"
        unit.faults, unit.faults_held = parse_uds_faults(answer)
    else:
        if answer.kind == "negative":
            unit.protocol = "UDS"
        older = elm.ask(unit.response_id, b"\x18\x02\xff\x00")
        if older.kind == "positive":
            unit.protocol = "KWP2000"
            unit.faults, unit.faults_held = parse_kwp_faults(older)
            answer = older
        else:
            unit.fault_note = f"fault codes not readable: {answer.describe()}"
    if answer.kind == "positive" and not answer.complete:
        unit.fault_note = (
            f"reply cut short ({len(answer.data)} of {answer.expected} bytes): "
            "the adapter did not keep the conversation going"
        )

    if unit.protocol == "KWP2000":
        ident = elm.ask(unit.response_id, b"\x1a\x86")
        if ident.kind == "positive":
            unit.identity["identification"] = readable(ident.data[2:])
        return
    for identifier, name in UDS_IDENTIFIERS.items():
        ident = elm.ask(unit.response_id, bytes([0x22, identifier >> 8, identifier & 0xFF]))
        if ident.kind == "none":
            break  # a unit that goes quiet will not answer the rest either
        if ident.kind == "positive" and len(ident.data) > 3:
            value = readable(ident.data[3:])
            unit.identity[name] = value if ident.complete else value + " (cut short)"


def describe_unit(unit: Unit) -> list[str]:
    identity = ", ".join(f"{name} {value}" for name, value in unit.identity.items())
    lines = [f"Unit {unit.address}  {unit.protocol}" + (f"  {identity}" if identity else "")]
    for fault in unit.faults:
        hint = f"  <- {fault.hint} (reported on related models)" if fault.hint else ""
        lines.append(f"    {fault.code}  {fault.label}  {fault.state}{hint}")
    if unit.faults_held is not None and unit.faults_held > len(unit.faults):
        lines.append(f"    ... {unit.faults_held} codes held, only {len(unit.faults)} received")
    if unit.fault_note:
        lines.append(f"    note: {unit.fault_note}")
    elif not unit.faults:
        lines.append("    no fault codes")
    return lines


def run_session(sock: socket.socket, options: Options, say=print, log=None) -> dict:
    """Run the whole check on a connected adapter and return everything found."""
    elm = Elm(sock, log)
    result: dict = {"units": [], "notes": []}
    try:
        elm.reset()
        result["adapter"] = elm.describe()
        say(f"Adapter: {result['adapter']['version']}   Battery: {result['adapter']['battery']}")
        elm.use_can_500k()
        caps = elm.detect_capabilities()
        result["capabilities"] = asdict(caps)
        say("Adapter features:")
        for name, present, why in (
            ("listen on one address", caps.receive_address, "reach non-engine units"),
            ("listen on a range", caps.filter_mask, "wide sweep"),
            ("flow control", caps.flow_control, "read more than one fault code per unit"),
            ("timeouts", caps.timeout, "wait for slow units"),
            ("raw monitoring", caps.raw_monitor, "check for background traffic"),
        ):
            say(f"  {'yes' if present else 'NO ':3}  {name} ({why})")
        if options.probe_only:
            return result

        elm.set_timeout(SCAN_TIMEOUT_MS)
        elm.select(0x7DF, 0x7E8)
        check = elm.request(b"\x01\x00")
        result["engine_answered"] = bool(check.from_id(0x7E8))
        if not result["engine_answered"]:
            detail = ", ".join(check.notes) or "no reply"
            say(f"The engine control unit did not answer ({detail}).")
            say("Check the ignition is on (key position 2) and the adapter is fully seated.")
            return result
        say("Network check: the engine control unit answered.")

        busy: set[int] = set()
        if options.listen_seconds > 0:
            busy, overflowed = elm.monitor(options.listen_seconds)
            result["background_addresses"] = sorted(busy)
            if overflowed:
                result["notes"].append("background traffic overflowed the adapter")
            say(
                "Background traffic: "
                + (f"{len(busy)} addresses in use; those are left alone" if busy else "none")
            )

        targets = [(pair, "standard") for pair in standard_targets()]
        if caps.can_listen_anywhere:
            targets += [(pair, "mercedes") for pair in mercedes_targets()]
        else:
            note = "adapter can only listen on engine addresses; other units are out of reach"
            result["notes"].append(note)
            say(f"Warning: {note}.")

        say(f"Looking for control units at {len(targets)} addresses...")
        units: list[Unit] = []
        for (request_id, response_id), scheme in targets:
            if request_id in busy or response_id in busy:
                continue
            answer = probe(elm, request_id, response_id, options.mask)
            if answer:
                unit = Unit(request_id, response_id, scheme, answer)
                units.append(unit)
                say(f"  {unit.address} answers")

        if options.wide:
            say("Wide sweep of the remaining addresses (takes a minute or two)...")
            skip = {request_id for (request_id, _), _ in targets} | busy | {0x7DF}
            skip |= set(range(0x7E8, 0x7F0)) | {unit.response_id for unit in units}
            units += sweep(elm, skip, busy, options.mask, say)

        say(f"Reading fault codes from {len(units)} units...")
        elm.set_timeout(READ_TIMEOUT_MS)
        for unit in units:
            read_unit(elm, unit, options.mask)
        result["units"] = [asdict(unit) for unit in units]

        say("")
        for unit in units:
            for line in describe_unit(unit):
                say(line)
        if units and all(unit.found_by == "standard" for unit in units) and caps.can_listen_anywhere:
            note = "only engine-side units answered; rerun with --wide to try every address"
            result["notes"].append(note)
            say(f"\nNote: {note}.")
        return result
    finally:
        elm.restore_defaults()


# --- Command line -----------------------------------------------------------

CONNECT_HINTS = {
    errno.EHOSTDOWN: "The adapter did not answer. Is it plugged into the car, with the ignition "
    "on, and within a few metres?",
    errno.ECONNREFUSED: "The adapter refused the connection. Pair it first (see README), or try "
    "--channel 2.",
    errno.ETIMEDOUT: "Connecting timed out. Check the adapter is powered and in range.",
    errno.EHOSTUNREACH: "The adapter is out of reach. Check it is powered and in range.",
}


class SerialPort:
    """A serial device, with the few socket methods the adapter code uses.

    macOS gives a paired Bluetooth adapter a serial port such as /dev/cu.OBDII;
    Python there cannot open a Bluetooth socket directly.
    """

    def __init__(self, path: str):
        import termios  # absent on Windows

        self._fd = os.open(path, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
        self._timeout: float | None = None
        try:
            settings = termios.tcgetattr(self._fd)
            settings[0] = 0  # input: no translation, no flow control
            settings[1] = 0  # output: no translation
            settings[2] = termios.CS8 | termios.CREAD | termios.CLOCAL
            settings[3] = 0  # no echo, no line editing
            settings[4] = settings[5] = termios.B38400
            settings[6][termios.VMIN] = 0
            settings[6][termios.VTIME] = 0
            termios.tcsetattr(self._fd, termios.TCSANOW, settings)
        except BaseException:
            os.close(self._fd)
            raise

    def settimeout(self, seconds: float | None) -> None:
        self._timeout = seconds

    def recv(self, size: int) -> bytes:
        ready, _, _ = select.select([self._fd], [], [], self._timeout)
        if not ready:
            raise TimeoutError("timed out")
        try:
            return os.read(self._fd, size)
        except BlockingIOError:
            raise TimeoutError("timed out") from None

    def sendall(self, data: bytes) -> None:
        remaining = memoryview(data)
        while remaining:
            _, ready, _ = select.select([], [self._fd], [], 5)
            if not ready:
                raise TimeoutError("The serial port stopped accepting data.")
            try:
                written = os.write(self._fd, remaining)
            except BlockingIOError:
                continue
            remaining = remaining[written:]

    def close(self) -> None:
        if self._fd >= 0:
            os.close(self._fd)
            self._fd = -1

    def __enter__(self) -> "SerialPort":
        return self

    def __exit__(self, *exc) -> None:
        self.close()


def connect(args: argparse.Namespace):
    if args.tcp:
        host, _, port = args.tcp.rpartition(":")
        return socket.create_connection((host, int(port)), timeout=10)
    if args.port:
        try:
            return SerialPort(args.port)
        except FileNotFoundError:
            raise ElmError(
                f"There is no serial port {args.port}. Pair the adapter first, then list the "
                "ports with: ls /dev/cu.*"
            ) from None
    if not hasattr(socket, "AF_BLUETOOTH"):
        raise ElmError(
            "This Python cannot open Bluetooth connections directly (normal on a Mac). "
            "Pair the adapter and use --port /dev/cu.<name> instead; see the README."
        )
    sock = socket.socket(socket.AF_BLUETOOTH, socket.SOCK_STREAM, socket.BTPROTO_RFCOMM)
    sock.settimeout(20)
    try:
        sock.connect((args.address, args.channel))
    except OSError as error:
        sock.close()
        hint = CONNECT_HINTS.get(error.errno, "")
        raise ElmError(f"Could not connect to {args.address}: {error.strerror}. {hint}".strip())
    return sock


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    parser.add_argument("address", nargs="?", help="Bluetooth address of the adapter, AA:BB:CC:DD:EE:FF")
    parser.add_argument("--channel", type=int, default=1, help="Bluetooth serial channel (default 1)")
    parser.add_argument("--port", metavar="PATH", help="serial port of a paired adapter, e.g. /dev/cu.OBDII on a Mac")
    parser.add_argument("--tcp", metavar="HOST:PORT", help="connect to a Wi-Fi adapter or the simulator")
    parser.add_argument("--wide", action="store_true", help="also try every address from 600 to 7FF")
    parser.add_argument("--probe-only", action="store_true", help="only report what the adapter supports")
    parser.add_argument("--no-listen", action="store_true", help="skip the background-traffic check")
    parser.add_argument(
        "--mask",
        type=lambda text: int(text, 16),
        default=DEFAULT_STATUS_MASK,
        help="fault status mask in hex (default 0D: current, pending, stored)",
    )
    args = parser.parse_args(argv)
    if sum(bool(choice) for choice in (args.address, args.port, args.tcp)) != 1:
        parser.error("give exactly one of: the adapter's Bluetooth address, --port PATH, --tcp HOST:PORT")

    options = Options(
        wide=args.wide,
        mask=args.mask,
        listen_seconds=0 if args.no_listen else 2.0,
        probe_only=args.probe_only,
    )
    LOG_DIR.mkdir(exist_ok=True)
    stamp = datetime.now().strftime("%Y%m%d-%H%M%S")
    log_path = LOG_DIR / f"slkdiag-{stamp}.log"
    json_path = LOG_DIR / f"slkdiag-{stamp}.json"
    try:
        sock = connect(args)
    except (ElmError, OSError, ValueError) as error:
        print(f"Error: {error}", file=sys.stderr)
        return 1
    status = 0
    with sock, open(log_path, "w") as log:
        try:
            result = run_session(sock, options, log=log)
            json_path.write_text(json.dumps(result, indent=2))
            print(f"\nResults: {json_path}")
        except (ElmError, OSError) as error:
            print(f"Error: {error}", file=sys.stderr)
            status = 1
        except KeyboardInterrupt:
            print("\nStopped.", file=sys.stderr)
            status = 130
    print(f"Full adapter log: {log_path}")
    return status


if __name__ == "__main__":
    sys.exit(main())
