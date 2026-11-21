import argparse
import os
import pty
import select
import socket
import sys
import threading
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "tests"))

import slkdiag  # noqa: E402
from fake_elm import FakeElm, FakeUnit, pretend_slk, uds_unit  # noqa: E402


def run(units=None, options=None, **adapter_kwargs):
    """Run a full session against a pretend adapter; returns (result, adapter, printed lines)."""
    adapter = FakeElm(units if units is not None else pretend_slk(), **adapter_kwargs)
    adapter.start()
    printed: list[str] = []
    options = options or slkdiag.Options(listen_seconds=0.2)
    with socket.create_connection(("127.0.0.1", adapter.port), timeout=5) as sock:
        result = slkdiag.run_session(sock, options, say=printed.append)
    return result, adapter, printed


def unit_at(result, request_id):
    matches = [unit for unit in result["units"] if unit["request_id"] == request_id]
    return matches[0] if matches else None


class FullSessionTests(unittest.TestCase):
    def test_finds_units_and_reads_every_fault_code(self):
        result, _, printed = run()

        self.assertEqual(
            sorted(unit["request_id"] for unit in result["units"]), [0x612, 0x672, 0x6C2, 0x7E0]
        )
        climate = unit_at(result, 0x672)
        self.assertEqual(climate["response_id"], 0x48E)
        self.assertEqual(climate["protocol"], "UDS")
        self.assertEqual(climate["identity"]["hardware part number"], "1729003304")
        self.assertEqual([f["code"] for f in climate["faults"]], ["D00800", "912F87", "918707"])
        self.assertEqual([f["state"] for f in climate["faults"]], ["current, stored"] * 2 + ["stored"])
        self.assertEqual(climate["faults"][0]["label"], "U1008-00")
        self.assertEqual(climate["faults"][1]["hint"], "front blower regulator malfunction")
        self.assertEqual(climate["fault_note"], "")
        self.assertEqual(unit_at(result, 0x612)["faults"], [])
        self.assertIn("    912F87  B112F-87  current, stored"
                      "  <- front blower regulator malfunction (reported on related models)", printed)

    def test_older_unit_is_read_with_the_older_request(self):
        result, _, _ = run()

        older = unit_at(result, 0x6C2)
        self.assertEqual(older["protocol"], "KWP2000")
        self.assertEqual([(f["code"], f["state"]) for f in older["faults"]], [("951D", "current")])
        self.assertEqual(older["identity"], {"identification": "KW2C3PE"})

    def test_adapter_without_flow_control_reports_a_cut_off_reply(self):
        result, _, printed = run(flow_control=False)

        climate = unit_at(result, 0x672)
        self.assertFalse(result["capabilities"]["flow_control"])
        self.assertEqual(climate["faults_held"], 3)
        self.assertEqual([(f["code"], f["status"]) for f in climate["faults"]], [("D00800", None)])
        self.assertIn("cut short", climate["fault_note"])
        self.assertIn("    ... 3 codes held, only 1 received", printed)

    def test_falls_back_to_filter_and_mask_when_receive_address_is_missing(self):
        result, _, _ = run(receive_address=False)

        self.assertFalse(result["capabilities"]["receive_address"])
        self.assertEqual(len(unit_at(result, 0x672)["faults"]), 3)

    def test_adapter_that_cannot_change_its_listening_address_only_reaches_the_engine(self):
        result, adapter, printed = run(receive_address=False, filter_mask=False)

        self.assertEqual([unit["request_id"] for unit in result["units"]], [0x7E0])
        self.assertTrue(any("out of reach" in line for line in printed))
        self.assertFalse(any(0x600 <= header < 0x7DF for header, _ in adapter.sent))

    def test_wide_sweep_finds_a_unit_outside_the_usual_scheme(self):
        result, _, _ = run(options=slkdiag.Options(wide=True, listen_seconds=0.2))

        found = unit_at(result, 0x743)
        self.assertEqual((found["response_id"], found["found_by"]), (0x4E8, "sweep"))
        self.assertEqual(found["identity"]["hardware part number"], "1729012200")

    def test_addresses_already_in_use_on_the_bus_are_never_transmitted_on(self):
        result, adapter, _ = run(background=("612 00 11 22 33 44 55 66 77", "3F1 01 02"))

        self.assertEqual(result["background_addresses"], [0x3F1, 0x612])
        self.assertIsNone(unit_at(result, 0x612))
        self.assertNotIn(0x612, [header for header, _ in adapter.sent])
        self.assertIsNotNone(unit_at(result, 0x672))

    def test_stops_with_advice_when_the_engine_does_not_answer(self):
        result, _, printed = run(units={})

        self.assertFalse(result["engine_answered"])
        self.assertEqual(result["units"], [])
        self.assertTrue(any("ignition" in line for line in printed))

    def test_only_read_only_requests_reach_the_bus(self):
        _, adapter, _ = run(options=slkdiag.Options(wide=True, listen_seconds=0.2))

        services = {payload[0] for _, payload in adapter.sent}
        self.assertLessEqual(services, set(slkdiag.READ_ONLY_SERVICES))

    def test_probe_only_sends_nothing_to_the_car(self):
        result, adapter, _ = run(options=slkdiag.Options(probe_only=True))

        self.assertEqual(adapter.sent, [])
        self.assertTrue(result["capabilities"]["flow_control"])


class SerialPortTests(unittest.TestCase):
    """The route a Mac uses: the paired adapter appears as a serial port."""

    def setUp(self):
        self.master, self.slave = pty.openpty()
        self.addCleanup(os.close, self.master)
        self.addCleanup(os.close, self.slave)
        self.path = os.ttyname(self.slave)

    def bridge_to(self, adapter):
        """Wire the far end of the serial port to a pretend adapter."""
        link = socket.create_connection(("127.0.0.1", adapter.port), timeout=5)
        self.addCleanup(link.close)

        def pump():
            try:
                while True:
                    ready, _, _ = select.select([self.master, link], [], [])
                    if self.master in ready:
                        link.sendall(os.read(self.master, 1024))
                    if link in ready:
                        data = link.recv(1024)
                        if not data:
                            return
                        os.write(self.master, data)
            except (OSError, ValueError):
                return

        threading.Thread(target=pump, daemon=True).start()

    def test_full_session_over_a_serial_port(self):
        adapter = FakeElm(pretend_slk())
        adapter.start()
        self.bridge_to(adapter)

        with slkdiag.SerialPort(self.path) as port:
            result = slkdiag.run_session(
                port, slkdiag.Options(listen_seconds=0.2), say=lambda line: None
            )

        climate = unit_at(result, 0x672)
        self.assertEqual([f["code"] for f in climate["faults"]], ["D00800", "912F87", "918707"])
        self.assertEqual(unit_at(result, 0x6C2)["protocol"], "KWP2000")

    def test_bytes_pass_through_unchanged_in_both_directions(self):
        with slkdiag.SerialPort(self.path) as port:
            port.settimeout(2)
            port.sendall(b"ATZ\r")
            self.assertEqual(os.read(self.master, 16), b"ATZ\r")
            os.write(self.master, b"OK\r\r>")
            self.assertEqual(port.recv(16), b"OK\r\r>")

    def test_recv_times_out_when_nothing_arrives(self):
        with slkdiag.SerialPort(self.path) as port:
            port.settimeout(0.05)
            with self.assertRaises(TimeoutError):
                port.recv(16)

    def test_missing_port_explains_how_to_find_one(self):
        args = argparse.Namespace(tcp=None, port="/dev/cu.not-there", address=None, channel=1)

        with self.assertRaisesRegex(slkdiag.ElmError, r"ls /dev/cu\.\*"):
            slkdiag.connect(args)


class SlowUnitTests(unittest.TestCase):
    def test_unit_that_asks_for_more_time_is_asked_again(self):
        calls = []

        def slow(payload):
            if payload[:2] == b"\x19\x02":
                calls.append(payload)
                if len(calls) == 1:
                    return b"\x7f\x19\x78"
                return b"\x59\x02\xff" + bytes.fromhex("912F8709")
            return uds_unit("1729003304")(payload)

        units = pretend_slk()
        units[0x672] = FakeUnit(0x48E, slow)
        result, _, _ = run(units=units)

        self.assertEqual(len(calls), 2)
        self.assertEqual([f["code"] for f in unit_at(result, 0x672)["faults"]], ["912F87"])

    def test_pending_and_final_answer_in_one_reply(self):
        def chatty(payload):
            if payload[:2] == b"\x19\x02":
                return [b"\x7f\x19\x78", b"\x59\x02\xff" + bytes.fromhex("D0080008")]
            return uds_unit("1729003304")(payload)

        units = pretend_slk()
        units[0x672] = FakeUnit(0x48E, chatty)
        result, _, _ = run(units=units)

        self.assertEqual([f["code"] for f in unit_at(result, 0x672)["faults"]], ["D00800"])


class ParsingTests(unittest.TestCase):
    def test_frame_lines(self):
        self.assertEqual(slkdiag.parse_frame_line("482 02 7E 00"), (0x482, b"\x02\x7e\x00"))
        self.assertEqual(slkdiag.parse_frame_line("0482 02 7E 00"), (0x482, b"\x02\x7e\x00"))
        self.assertEqual(slkdiag.parse_frame_line("482027E00"), (0x482, b"\x02\x7e\x00"))
        self.assertEqual(slkdiag.parse_frame_line("7E8 03 7F 19 11 <DATA ERROR"), (0x7E8, b"\x03\x7f\x19\x11"))
        for text in ("NO DATA", "CAN ERROR", "FB ERROR", "OK", "?", "12.4V", "ELM327 v1.5", "41 00 BE 3F"):
            self.assertIsNone(slkdiag.parse_frame_line(text), text)

    def test_reassembles_a_long_message_and_drops_padding(self):
        frames = [
            bytes.fromhex("100F5902FFD00800"),
            bytes.fromhex("2109912F87099187"),
            bytes.fromhex("220708AAAAAAAAAA"),
        ]
        (message,) = slkdiag.reassemble(frames)

        self.assertTrue(message.complete)
        self.assertEqual(message.data.hex().upper(), "5902FFD0080009912F870991870708")

    def test_letter_form(self):
        self.assertEqual(slkdiag.letter_form(bytes.fromhex("918707")), "B1187-07")
        self.assertEqual(slkdiag.letter_form(bytes.fromhex("D00800")), "U1008-00")
        self.assertEqual(slkdiag.letter_form(bytes.fromhex("0301")), "P0301")

    def test_mercedes_addresses_match_the_known_pairs(self):
        pairs = dict(slkdiag.mercedes_targets())

        self.assertEqual((pairs[0x612], pairs[0x632], pairs[0x6B2]), (0x482, 0x486, 0x496))
        self.assertTrue(all(request <= 0x7FF for request in pairs))


class ReadOnlyGuardTests(unittest.TestCase):
    def test_refuses_anything_that_could_change_the_car(self):
        for request in ("14FFFFFF", "1003", "2E0100AA", "2F010003", "3101FF00", "1101", "2701"):
            with self.assertRaises(slkdiag.ReadOnlyViolation, msg=request):
                slkdiag.ensure_read_only(bytes.fromhex(request))

    def test_allows_the_requests_the_tool_uses(self):
        for request in ("3E00", "19020D", "1802FF00", "22F111", "1A86", "0100"):
            slkdiag.ensure_read_only(bytes.fromhex(request))


if __name__ == "__main__":
    unittest.main()
