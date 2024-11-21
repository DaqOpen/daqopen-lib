import unittest
import numpy as np
from daqopen.duedaq import DueDaq

class TestDueDaqSimBasic(unittest.TestCase):

    def setUp(self):
        """Set up the DueDaq instance using simulation mode before each test."""
        self.channels = ["A0", "A1", "A2"]  # Test with multiple channels
        self.daq = DueDaq(
            channels=self.channels,
            serial_port_name="SIM",  # Use simulation mode
            samplerate=10000.0,  # Desired sample rate for testing
            differential=False,  # Single-ended mode
            gain="SGL_1X",  # Default gain
            offset_enabled=False,  # No offset mode
            extend_to_int16=False,  # Test default
            realtime_sim=False # Shorter delay for testing
        )

    def test_start_acquisition(self):
        """Test that data acquisition starts correctly."""
        self.daq.start_acquisition()  # Start the acquisition
        self.assertEqual(self.daq._acq_state, "running")  # Ensure acquisition is running
        self.assertGreaterEqual(self.daq._num_frames_read, 0)  # Check that frames have started reading
        self.daq.stop_acquisition()
        self.assertEqual(self.daq._acq_state, "stopped")  # Ensure acquisition has stopped

    def test_read_data(self):
        """Test that data can be read from the DAQ system in simulation mode."""
        self.daq.start_acquisition()  # Start acquisition
        data = self.daq.read_data()  # Read a block of data
        self.daq.stop_acquisition()
        self.assertIsInstance(data, np.ndarray)  # Ensure the data is a NumPy array
        self.assertEqual(data.shape[1], len(self.channels))  # Ensure the number of channels is correct
        self.assertEqual(data.shape[0], self.daq._samples_per_block_channel)  # Check block size per channel
        self.assertTrue(np.all(data >= 0))  # Ensure that data values are in expected range
        

    def test_acquisition_flow(self):
        """Test the complete flow from start, reading data, and stopping acquisition."""
        self.daq.start_acquisition()  # Start acquisition
        data = self.daq.read_data()  # Read a block of data
        self.assertIsNotNone(data)  # Ensure data is not None
        self.daq.stop_acquisition()  # Stop acquisition
        self.assertEqual(self.daq._acq_state, "stopped")  # Ensure acquisition has stopped

class TestDueDaqSimAdv(unittest.TestCase):
    
    def test_set_samplerate_single_channel(self):
        my_daq = DueDaq(
            channels=["A1"],
            serial_port_name="SIM",  # Use simulation mode
            samplerate=10000.0,  # Desired sample rate for testing
            realtime_sim=False # Shorter delay for testing
        )
        self.assertEqual(my_daq.samplerate, 10000)
        self.assertEqual(my_daq._adc_prescal, 199)

    def test_set_samplerate_single_channel_low(self):
        my_daq = DueDaq(
            channels=["A1"],
            serial_port_name="SIM",  # Use simulation mode
            samplerate=1000.0,  # Desired sample rate for testing
            realtime_sim=False # Shorter delay for testing
        )
        self.assertEqual(my_daq.samplerate, 7812.5)
        self.assertEqual(my_daq._adc_prescal, 255)
    
       


if __name__ == "__main__":
    unittest.main()
