import unittest
from pathlib import Path

from owlet.protocol import (
    ChecksumError,
    Position,
    SyntaxError_,
    checksum,
    decode,
    encode,
    format_number,
    parse_position,
)

VECTORS = Path(__file__).resolve().parents[3] / "docs" / "protocol_vectors.txt"


class ChecksumTest(unittest.TestCase):
    def test_known_values(self):
        self.assertEqual(checksum("PING"), 0x10)
        self.assertEqual(checksum("MOVE,30,-15"), 0x3B)


class EncodeTest(unittest.TestCase):
    def test_bare_command(self):
        self.assertEqual(encode("PING"), b"$PING*10\r\n")

    def test_numbers(self):
        self.assertEqual(encode("MOVE", 30, -15), b"$MOVE,30,-15*3B\r\n")
        self.assertEqual(encode("MOVE", 12.5, -0.25), b"$MOVE,12.5,-0.25*3D\r\n")

    def test_number_formatting(self):
        self.assertEqual(format_number(30.0), "30")
        self.assertEqual(format_number(-0.001), "0")
        self.assertEqual(format_number(1.239), "1.24")

    def test_rejects_bad_name(self):
        with self.assertRaises(ValueError):
            encode("move")


class DecodeTest(unittest.TestCase):
    def test_shared_vectors(self):
        checked = 0
        for line in VECTORS.read_text().splitlines():
            if not line or line.startswith("#"):
                continue
            expect, frame = line.split(None, 1)
            with self.subTest(frame=frame):
                if expect == "OK":
                    decode(frame)
                elif expect == "CHK":
                    self.assertRaises(ChecksumError, decode, frame)
                else:
                    self.assertRaises(SyntaxError_, decode, frame)
            checked += 1
        self.assertGreater(checked, 20)

    def test_fields(self):
        s = decode("$POS,12.5,-3.0,30.0,-15.0,M*1F\r\n")
        self.assertEqual(s.name, "POS")
        self.assertEqual(parse_position(s), Position(12.5, -3.0, 30.0, -15.0, "M"))

    def test_round_trip(self):
        frame = encode("NUDGE", 1.5, -2).decode()
        s = decode(frame)
        self.assertEqual((s.name, s.fields), ("NUDGE", ("1.5", "-2")))


if __name__ == "__main__":
    unittest.main()
