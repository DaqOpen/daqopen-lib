import unittest
import sys
import os
import time
import numpy as np

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
sys.path.append(os.path.dirname(SCRIPT_DIR))

from daqopen.daqinfo import DaqInfo
from daqopen.channelbuffer import AcqBuffer, AcqBufferPool

class TestChannelBuffer(unittest.TestCase):
    def test_acq_buffer_simple(self):
        my_acq_buffer_1 = AcqBuffer(100, sample_delay=1)
        my_acq_buffer_2 = AcqBuffer(100, sample_delay=0)
        data = np.arange(220)
        for idx in range(20):
            # Put one sample aligned data into buffer
            my_acq_buffer_1.put_data(data[idx*11:(idx+1)*11])
            my_acq_buffer_2.put_data(data[idx*11:(idx+1)*11] - 1)
            # Read out the same regions
            buffer1_value = my_acq_buffer_1.read_data_by_index(idx*11,(idx+1)*11)
            buffer2_value = my_acq_buffer_2.read_data_by_index(idx*11,(idx+1)*11)
            # Only Test after second run (first is unequal)
            if idx > 0:
                self.assertIsNone(np.testing.assert_array_equal(buffer1_value, buffer2_value))

    def test_acq_buffer_pool(self):
        # Create DaqInfo Object
        info_dict = {
                    "board": {"type": "duedaq", "samplerate": 48000},
                    "channel": {"U1": {"gain": 1.0, "offset": 1.0, "delay": 0, "unit": "V", "ai_pin": "A0"},
                                "U2": {"gain": 2.0, "offset": 2.0, "delay": 0, "unit": "V", "ai_pin": "A1"}}}
        daq_info = DaqInfo.from_dict(info_dict)
        # Take actual time as starttime
        start_timestamp_us = int(time.time()*1e6)
        # Create Acqusitionbuffer Pool
        my_acq_pool = AcqBufferPool(daq_info, data_columns={"A0": 0, "A1": 1}, size=200, start_timestamp_us=start_timestamp_us)
        self.assertEqual(my_acq_pool.channel["U1"].sample_delay, 0)
        data_matrix = np.ones((100,2))
        data_matrix[:,0] *= np.arange(100)
        data_matrix[:,1] *= np.arange(100)
        # fill with data
        my_acq_pool.put_data(data_matrix)
        # timestamp at most recent sample
        ts_us = int(start_timestamp_us+1e6*data_matrix.shape[0]/daq_info.board.samplerate)
        my_acq_pool.add_timestamp(ts_us, data_matrix.shape[0])
        # read values
        u1_val = my_acq_pool.channel["U1"].read_data_by_index(10,20)
        u2_val = my_acq_pool.channel["U2"].read_data_by_index(10,20)
        # Check values
        self.assertIsNone(np.testing.assert_array_equal(data_matrix[10:20,0]*1-1, u1_val))
        self.assertIsNone(np.testing.assert_array_equal(data_matrix[10:20,1]*2-2, u2_val))

    def test_acq_buffer_pool_samplerate(self):
        # Create DaqInfo Object
        info_dict = {
                    "board": {"type": "duedaq", "samplerate": 10000},
                    "channel": {"U1": {"gain": 1.0, "offset": 1.0, "delay": 0, "unit": "V", "ai_pin": "A0"},
                                "U2": {"gain": 2.0, "offset": 2.0, "delay": 0, "unit": "V", "ai_pin": "A1"}}}
        daq_info = DaqInfo.from_dict(info_dict)
        # Create Acqusitionbuffer Pool
        my_acq_pool = AcqBufferPool(daq_info, data_columns={"A0": 0, "A1": 1}, size=200)
        # Prepare Data
        data_matrix = np.ones((100,2))
        data_matrix[:,0] *= np.arange(100)
        data_matrix[:,1] *= np.arange(100)
        time_data = np.arange(0.000, 0.001, step=1.0/daq_info.board.samplerate)
        # fill with data
        my_acq_pool.put_data_with_samplerate(data_matrix, daq_info.board.samplerate)
        # read timestamps
        ts = my_acq_pool.time.read_data_by_index(0,10)
        # Check values
        self.assertIsNone(np.testing.assert_array_almost_equal(time_data, ts/1e6))

if __name__ == "__main__":
    unittest.main()
