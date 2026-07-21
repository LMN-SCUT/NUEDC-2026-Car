import pathlib
import sys
import unittest

PROTOCOL_PYTHON = pathlib.Path(__file__).resolve().parents[2] / "shared" / "protocol" / "python"
sys.path.insert(0, str(PROTOCOL_PYTHON))

import vision_link as vl


STANDARD_VECTOR = bytes.fromhex(
    "AA 55 01 01 05 10 E8 03 00 00 40 01 F0 00 "
    "0A 00 FB FF 5A 01 00 00 BF BE"
)


class ProtocolTests(unittest.TestCase):
    def parse_all(self, data):
        parser = vl.StreamParser()
        frames = []
        for byte in data:
            frame = parser.push(byte)
            if frame is not None:
                frames.append(frame)
        return frames

    def test_standard_vector(self):
        frame = vl.pack_observation(
            5, 1000, 320, 240, 10, -5, 90,
            vl.FLAG_TARGET_VALID, 0, 0,
        )
        self.assertEqual(frame, STANDARD_VECTOR)
        self.assertEqual(vl.crc16_ccitt_false(frame[2:-2]), 0xBEBF)

    def test_round_trip(self):
        frames = self.parse_all(STANDARD_VECTOR)
        self.assertEqual(len(frames), 1)
        version, message_type, seq, payload = frames[0]
        self.assertEqual((version, message_type, seq), (1, 1, 5))
        observation = vl.decode_observation(payload)
        self.assertEqual(observation["error_x"], 10)
        self.assertEqual(observation["error_y"], -5)

    def test_invalid_target_is_safe(self):
        raw = vl.pack_observation(1, 50, 123, 456, 10, 20, 99, 0, 7, 2)
        payload = self.parse_all(raw)[0][3]
        observation = vl.decode_observation(payload)
        self.assertEqual(observation["confidence"], 0)
        self.assertEqual(observation["target_id"], 0xFF)
        self.assertEqual(
            (observation["center_x"], observation["center_y"],
             observation["error_x"], observation["error_y"]),
            (0, 0, 0, 0),
        )

    def test_bad_crc_is_rejected_and_next_frame_recovers(self):
        broken = bytearray(STANDARD_VECTOR)
        broken[10] ^= 0x80
        stream = b"\x00\xAA\x10" + bytes(broken) + STANDARD_VECTOR
        frames = self.parse_all(stream)
        self.assertEqual(len(frames), 1)
        self.assertEqual(frames[0][2], 5)

    def test_command_round_trip(self):
        raw = vl.pack_command(8, 4, 0, 320, 240, 0)
        frame = self.parse_all(raw)[0]
        command = vl.decode_command(frame[3])
        self.assertEqual(command["command_id"], 4)
        self.assertEqual((command["arg0"], command["arg1"]), (320, 240))


if __name__ == "__main__":
    unittest.main()
