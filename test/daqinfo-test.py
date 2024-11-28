import unittest
import sys
import os

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
sys.path.append(os.path.dirname(SCRIPT_DIR))

from daqopen.daqinfo import DaqInfo, InputInfo, BoardInfo

def is_subset(subset_dict, large_dict):
    """
    Prüft rekursiv, ob `subset_dict` ein Teil von `large_dict` ist, inklusive verschachtelter Dictionaries.
    """
    for key, value in subset_dict.items():
        if key not in large_dict:
            return False
        if isinstance(value, dict):  # Wenn der Wert selbst ein Dictionary ist, rekursiv prüfen
            if not isinstance(large_dict[key], dict):
                return False
            if not is_subset(value, large_dict[key]):
                return False
        elif large_dict[key] != value:  # Vergleiche für nicht-Dict-Werte
            return False
    return True

class TestDaqInfo(unittest.TestCase):
    def test_from_dict_to_dict(self):
        input_data = {
            "board": {
                "type": "duedaq",
                "samplerate": 1000.0,
                "differential": True,
                "gain": "SGL_X1",
                "offset_enabled": True,
                "adc_range": (0, 4095),
                "adc_clock_gain": 0.9998
            },
            "channel": {
                "ch1": {"gain": 2.0, "offset": 1.0, "delay": 10, "unit": "A", "ai_pin": "A0"},
                "ch2": {"gain": 1.5, "offset": 0.5, "delay": 5, "unit": "V", "ai_pin": "A1"},
            }
        }

        daq_info = DaqInfo.from_dict(input_data)
        output_data = daq_info.to_dict()

        #self.assertEqual(input_data["board"], output_data["board"])
        #self.assertEqual(input_data["channel"], output_data["channel"])
        self.assertTrue(is_subset(input_data, output_data))

    def test_apply_sensor_to_channel(self):

        daq_info = DaqInfo(
            board_info=BoardInfo(type="duedaq", samplerate=10000),
            channel_info={
                "ch1": InputInfo(gain=2.0, offset=1.0, delay=10, sensor="a"),
                "ch2": InputInfo(gain=1.5, offset=0.5, delay=5),
            },
            sensor_info = {
                "a": InputInfo(gain=0.5, offset=2.0, delay=3)
            }
        )

        new_channel_info = daq_info.get_channel_info_with_sensor()

        self.assertEqual(new_channel_info["ch1"].gain, 1.0)  # 2.0 * 0.5
        self.assertEqual(new_channel_info["ch1"].offset, 2.5)  # (1.0 * 0.5) + 2.0
        self.assertEqual(new_channel_info["ch1"].delay, 13)  # 10 + 3

    def test_with_sensor(self):
        input_data = {
            "board": {
                "type": "duedaq",
                "samplerate": 1000.0,
                "differential": True,
                "gain": "SGL_X1",
                "offset_enabled": True,
                "adc_range": (0, 4095),
                "adc_clock_gain": 0.9998
            },
            "channel": {
                "ch1": {"gain": 2.0, "offset": 1.0, "delay": 10, "unit": "A", "ai_pin": "A0", "sensor": "cur1"},
                "ch2": {"gain": 1.5, "offset": 0.5, "delay": 5, "unit": "V", "ai_pin": "A1", "sensor": "volt1"},
            },
            "sensor": {
                "cur1": {"gain": 2000.0, "offset": 0.0, "delay": 10, "unit": "A"},
                "volt1": {"gain": 2000.0, "offset": 0.0, "delay": 10, "unit": "A"}
            }
        }

        daq_info = DaqInfo.from_dict(input_data)
        output_data = daq_info.to_dict()

        #self.assertEqual(input_data, output_data)
        self.assertTrue(is_subset(input_data, output_data))

if __name__ == "__main__":
    unittest.main()
