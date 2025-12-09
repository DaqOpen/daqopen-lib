import unittest
import sys
import os
import time
import numpy as np

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
sys.path.append(os.path.dirname(SCRIPT_DIR))

from daqopen.daqinfo import DaqInfo
from daqopen.channelbuffer import AcqBuffer, AcqBufferPool, DataChannelBuffer

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

class TestDataChannelBuffer(unittest.TestCase):
    def setUp(self):
        """
        Set up test cases with a small buffer for testing.
        """
        self.buffer = DataChannelBuffer(name="TestBuffer", size=10, sample_dimension=1, agg_type="rms")

    def test_put_data_single(self):
        """
        Test inserting a single data sample into the buffer.
        """
        self.buffer.put_data_single(1, 2.0)
        self.assertEqual(self.buffer.sample_count, 1)
        self.assertEqual(self.buffer.last_sample_value, 2.0)
        self.assertEqual(self.buffer.last_sample_acq_sidx, 1)
        self.assertTrue((self.buffer._data[0] == 2.0))

    def test_put_data_multi(self):
        """
        Test inserting multiple data sample into the buffer.
        """
        self.buffer.put_data_multi(np.array([1,2,3,4,5]), np.array([1,2,3,4,5])*2)
        self.assertEqual(self.buffer.sample_count, 5)
        self.assertEqual(self.buffer.last_sample_value, 10.0)
        self.assertEqual(self.buffer.last_sample_acq_sidx, 5)
        self.assertTrue((self.buffer._data[3] == 8.0))

        self.buffer.put_data_multi(np.array([6,7,8,9,10,11]), np.array([6,7,8,9,10,11])*2)
        self.assertEqual(self.buffer.sample_count, 11)
        self.assertEqual(self.buffer.last_sample_value, 22.0)
        self.assertEqual(self.buffer.last_sample_acq_sidx, 11)
        self.assertTrue((self.buffer._data[0] == 22.0))

    def test_circular_buffer_behavior(self):
        """
        Test circular buffer overwriting when the buffer is full.
        """
        for i in range(15):  # Exceed buffer size
            self.buffer.put_data_single(i, float(i))
        self.assertEqual(self.buffer.sample_count, 15)
        self.assertEqual(self.buffer.last_sample_value, 14.0)
        self.assertEqual(self.buffer.last_sample_acq_sidx, 14)
        self.assertTrue((self.buffer._data[0] == 10.0))  # First value after overwrite
        self.assertTrue((self.buffer._data[-1] == 9.0))

    def test_read_data_by_acq_sidx(self):
        """
        Test reading data by acquisition indices.
        """
        for i in range(5):
            self.buffer.put_data_single((i * 10), i)
        data, ts = self.buffer.read_data_by_acq_sidx(0, 40)
        np.testing.assert_array_equal(data, [0.0, 1.0, 2.0, 3.0])
        np.testing.assert_array_equal(ts, [0, 10, 20, 30])

    def test_read_data_by_acq_sidx_2(self):
        """
        Test reading data by acquisition indices.
        """
        self.buffer.put_data_single(999, 1)
        self.buffer.put_data_single(1000, 2)
        self.buffer.put_data_single(1999, 3)
        data, ts = self.buffer.read_data_by_acq_sidx(0, 1000)
        np.testing.assert_array_equal(data, [1.0])
        np.testing.assert_array_equal(ts, [999])

    def test_read_data_by_acq_sidx_include_next(self):
        """
        Test reading data by acquisition indices and include next=True
        """
        self.buffer.put_data_single(1001, 1)
        data, ts = self.buffer.read_data_by_acq_sidx(900, 1000, include_next=True)
        np.testing.assert_array_equal(data, [1.0])
        np.testing.assert_array_equal(ts, [1001])
        data, ts = self.buffer.read_data_by_acq_sidx(1002, 1100, include_next=True)
        np.testing.assert_array_equal(data, [])
        np.testing.assert_array_equal(ts, [])
        self.buffer.put_data_single(2001, 2)
        self.buffer.put_data_single(3001, 3)

    def test_read_agg_data_by_acq_sidx_include_next(self):
        """
        Test reading data by acquisition indices and include next=True
        """
        self.buffer.put_data_single(1001, 1)
        data, ts = self.buffer.read_agg_data_by_acq_sidx(900, 1000, include_next=True)
        np.testing.assert_array_equal(data, [1.0])
        np.testing.assert_array_equal(ts, [1001])
        data, ts = self.buffer.read_agg_data_by_acq_sidx(1002, 1100, include_next=True)
        np.testing.assert_array_equal(data, [])
        np.testing.assert_array_equal(ts, [])
        self.buffer.put_data_single(2001, 2)
        self.buffer.put_data_single(3001, 3)

    def test_read_data_wraparound(self):
        """
        Test reading data when the circular buffer has wrapped around.
        """
        for i in range(15):  # Exceed buffer size
            self.buffer.put_data_single(i, float(i))
        data, ts = self.buffer.read_data_by_acq_sidx(12, 15)
        np.testing.assert_array_equal(data, [12.0, 13.0, 14.0])
        np.testing.assert_array_equal(ts, [12, 13, 14])

    def test_read_agg_data_rms(self):
        """
        Test reading RMS aggregated data.
        """
        for i in range(5):
            self.buffer.put_data_single(i, float(i))
        result, _ = self.buffer.read_agg_data_by_acq_sidx(1, 5)
        expected_rms = np.sqrt(np.mean(np.square([1.0, 2.0, 3.0, 4.0])))
        self.assertAlmostEqual(result, expected_rms, places=6)

    def test_read_agg_data_max(self):
        """
        Test reading max aggregated data.
        """
        buffer = DataChannelBuffer(name="MaxBuffer", size=10, sample_dimension=1, agg_type="max")
        for i in range(5):
            buffer.put_data_single(i, float(i))
        result, _ = buffer.read_agg_data_by_acq_sidx(1, 5)
        self.assertEqual(result, 4.0)

    def test_read_agg_data_mean(self):
        """
        Test reading mean aggregated data.
        """
        buffer = DataChannelBuffer(name="MeanBuffer", size=10, sample_dimension=1, agg_type=None)
        for i in range(5):
            buffer.put_data_single(i, float(i))
        result, _ = buffer.read_agg_data_by_acq_sidx(1, 5)
        self.assertEqual(result, 2.5)

    def test_read_agg_data_phi_0(self):
        """
        Test reading phi aggregated data near zero
        """
        buffer = DataChannelBuffer(name="MeanBuffer", size=10, sample_dimension=1, agg_type="phi")
        for i in range(5):
            buffer.put_data_single(i, float(i-2.5))
        result, _ = buffer.read_agg_data_by_acq_sidx(1, 5)
        self.assertEqual(result, 0)

    def test_read_agg_data_phi_180(self):
        """
        Test reading phi aggregated data near +-180°
        """
        buffer = DataChannelBuffer(name="MeanBuffer", size=10, sample_dimension=1, agg_type="phi")
        for i in range(5):
            buffer.put_data_single(i, float(i-182.5))
        result, _ = buffer.read_agg_data_by_acq_sidx(1, 5)
        self.assertEqual(result, 0)


    def test_read_agg_data_empty(self):
        """
        Test reading aggregated data with no matching indices.
        """
        result, _ = self.buffer.read_agg_data_by_acq_sidx(100, 200)
        self.assertIsNone(result)

    def test_read_data_buffer_edge(self):
        """
        Test reading data when the cursor is on the buffer end.
        """
        #self.buffer.put_data_multi(np.arange(0,10), np.arange(0,10, dtype=np.float32))
        for i in range(10):
            self.buffer.put_data_single(i, float(i))
        self.assertEqual(self.buffer.last_write_idx, 10) 
        data, ts = self.buffer.read_data_by_acq_sidx(0, 10, include_next=True)
        np.testing.assert_array_equal(data, np.arange(0, 10, dtype=np.float32))
        np.testing.assert_array_equal(ts, np.arange(0, 10))


if __name__ == "__main__":
    unittest.main()
