import unittest
import sys
import os

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
sys.path.append(os.path.dirname(SCRIPT_DIR))

from daqopen.daqinfo import DaqInfo, InputInfo, BoardInfo

class TestDaqInfo(unittest.TestCase):
    def test_from_dict_to_dict(self):
        input_data = {
            "board": {
                "type": "duedaq",
                "samplerate": 1000.0,
                "differential": True,
                "gain": "SGL_X1",
                "offset_enabled": True
            },
            "channel": {
                "ch1": {"gain": 2.0, "offset": 1.0, "delay": 10, "unit": "A", "ai_pin": "A0"},
                "ch2": {"gain": 1.5, "offset": 0.5, "delay": 5, "unit": "V", "ai_pin": "A1"},
            },
        }

        daq_info = DaqInfo.from_dict(input_data)
        output_data = daq_info.to_dict()

        self.assertEqual(input_data, output_data)

    def test_apply_sensor_to_channel(self):
        sensor_info = InputInfo(gain=0.5, offset=2.0, delay=3)

        daq_info = DaqInfo(
            board_info=BoardInfo(type="duedaq", samplerate=10000),
            channel_info={
                "ch1": InputInfo(gain=2.0, offset=1.0, delay=10),
                "ch2": InputInfo(gain=1.5, offset=0.5, delay=5),
            },
        )

        daq_info.apply_sensor_to_channel("ch1", sensor_info)

        self.assertEqual(daq_info.channel["ch1"].gain, 1.0)  # 2.0 * 0.5
        self.assertEqual(daq_info.channel["ch1"].offset, 2.5)  # (1.0 * 0.5) + 2.0
        self.assertEqual(daq_info.channel["ch1"].delay, 13)  # 10 + 3

if __name__ == "__main__":
    unittest.main()
