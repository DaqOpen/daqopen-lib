import unittest
import numpy as np
import time
import sys
import os

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
sys.path.append(os.path.dirname(SCRIPT_DIR))

from daqopen.daqzmq import DaqPublisher, DaqSubscriber
from daqopen.daqinfo import DaqInfo, BoardInfo, InputInfo

test_daqinfo = DaqInfo(
            board_info=BoardInfo(type="duedaq", samplerate=10000),
            channel_info={
                "ch1": InputInfo(gain=2.0, offset=1.0, delay=10),
                "ch2": InputInfo(gain=1.5, offset=0.5, delay=5),
            })
test_data_columns = {"ch1": 0, "ch2": 1}

class TestDaqZmq(unittest.TestCase):
    def test_simple_transfer(self):
        my_pub = DaqPublisher(port=50012, daq_info=test_daqinfo, data_columns=test_data_columns)
        my_sub = DaqSubscriber(port=50012, init_daqinfo=False)
        time.sleep(0.1)

        daq_data = np.ones((100,1), dtype=np.int16)
        daq_data[:,0] *= np.arange(100)

        ts = time.time()

        my_pub.send_data(daq_data, 0, ts, False)
        recv_data = my_sub.recv_data()

         # Check values
        self.assertEqual(ts, my_sub.timestamp)
        self.assertEqual(0, my_sub.packet_num)
        self.assertIsNone(np.testing.assert_array_equal(daq_data, recv_data))

        my_pub.terminate()
        my_sub.terminate()

    def test_transfer_start_gap(self):
        # DaqPublisher Instance
        my_pub = DaqPublisher(port=50012, daq_info=test_daqinfo, data_columns=test_data_columns)        # Create Data Object
        daq_data = np.ones((100,1), dtype=np.int16)
        daq_data[:,0] *= np.arange(100)
        # Send first data
        ts = time.time()
        my_pub.send_data(daq_data, 0, ts, False)

        # DaqSubscriber Instance
        my_sub = DaqSubscriber(port=50012, init_daqinfo=False)
        time.sleep(0.1)
        # Modify Data Object
        daq_data[:,0] += np.arange(100)
        ts = time.time()
        # Send second data
        my_pub.send_data(daq_data, 1, ts, False)

        recv_data = my_sub.recv_data()

         # Check values (only second message must be received)
        self.assertEqual(ts, my_sub.timestamp)
        self.assertEqual(1,my_sub.packet_num)
        self.assertIsNone(np.testing.assert_array_equal(daq_data, recv_data))

        my_pub.terminate()
        my_sub.terminate()

    def test_multi_packet_transfer(self):
        my_pub = DaqPublisher(port=50012, daq_info=test_daqinfo, data_columns=test_data_columns)
        my_sub = DaqSubscriber(port=50012, init_daqinfo=False)
        time.sleep(0.1)

        daq_data = np.ones((100,1), dtype=np.int16)
        daq_data[:,0] *= np.arange(100)
        
        for pkg_idx in range(100):
            ts = time.time()
            daq_data += 100

            my_pub.send_data(daq_data, pkg_idx, ts, False)
            recv_data = my_sub.recv_data()

            # Check values
            self.assertEqual(ts, my_sub.timestamp)
            self.assertEqual(pkg_idx, my_sub.packet_num)
            self.assertIsNone(np.testing.assert_array_equal(daq_data, recv_data))

        my_pub.terminate()
        my_sub.terminate()

    def test_burst_transfer(self):
        my_pub = DaqPublisher(port=50012, daq_info=test_daqinfo, data_columns=test_data_columns)
        my_sub = DaqSubscriber(port=50012, init_daqinfo=False)
        time.sleep(0.1)

        daq_data = np.ones((100,1), dtype=np.int16)
        daq_data[:,0] *= np.arange(100)

        ts_list = []
        data_list = []
        
        # Send all
        for pkg_idx in range(100):
            ts = time.time()
            ts_list.append(ts)
            daq_data += 100
            data_list.append(daq_data.copy())
            my_pub.send_data(daq_data, pkg_idx, ts, False)

        # Receive all
        for pkg_idx in range(100):
            recv_data = my_sub.recv_data()
            # Check values
            self.assertEqual(ts_list[pkg_idx], my_sub.timestamp)
            self.assertEqual(pkg_idx, my_sub.packet_num)
            self.assertIsNone(np.testing.assert_array_equal(data_list[pkg_idx], recv_data))

        my_pub.terminate()
        my_sub.terminate()

if __name__ == "__main__":
    unittest.main()
