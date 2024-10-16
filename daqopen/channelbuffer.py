# daqopen/channelbuffer.py

"""
Module for buffering acquisition and channel data.

This module provides classes for buffering data acquired from data acquisition systems. It includes 
`AcqBufferPool` for managing buffers for multiple channels with a single timebase, and `AcqBuffer` 
for implementing a cyclic buffer for individual channels.

## Usage

The primary classes in this module are `AcqBufferPool` and `AcqBuffer`. `AcqBufferPool` facilitates 
buffering data for multiple channels, allowing for convenient storage and retrieval of both channel 
data and timestamps. `AcqBuffer` is used for buffering data in a cyclic manner, with support for scaling 
and delaying the data.

Examples:
    Create an `AcqBufferPool` instance and add data with timestamps:

    >>> from channelbuffer import AcqBufferPool
    >>> my_daq_info = DaqInfo(...)  # Assume this is an initialized DaqInfo object
    >>> buffer_pool = AcqBufferPool(daq_info=my_daq_info)
    >>> buffer_pool.put_data_with_timestamp(data_array, timestamp_us=12345678)

Classes:
    AcqBufferPool: Manages buffers for multiple channels with a single timebase.
    AcqBuffer: Implements a cyclic buffer for individual channels.

"""

import numpy as np
from .daqinfo import DaqInfo
from copy import deepcopy

class AcqBufferPool(object):
    """
    Manages buffers for multiple channels with a single timebase.

    `AcqBufferPool` facilitates the buffering of data for more than one channel, all sharing a common 
    timebase. It provides methods for adding data and timestamps, maintaining synchronization across 
    channels.

    Attributes:
        channel (dict): A dictionary mapping channel names to their respective `AcqBuffer` instances.
        time (AcqBuffer): Buffer for storing timestamps.
        actual_sidx (int): Index of the last sample added to the buffers.
        _daq_info (DaqInfo): Information about the DAQ system, including channel configurations.
        _data_columns (dict): Maps analog input pins to data column indices.
        _buffer_size (int): Number of samples in each channel's buffer.
        _last_timestamp_us (int): Stores the last timestamp in microseconds.
        _time_batch_array (np.array): Array used to generate timestamp batches.

    Methods:
        put_data(data: np.array): Adds data to the channel buffers.
        add_timestamp(timestamp_us: int, num_samples: int): Adds timestamps to the time buffer.
        put_data_with_timestamp(data: np.array, timestamp_us: int): Adds channel data and timestamps to the buffers.
        put_data_with_samplerate(data: np.array, samplerate: float): Adds data and updates timestamps using sample rate.

    Examples:
        >>> buffer_pool = AcqBufferPool(daq_info=my_daq_info, data_columns=my_data_columns, size=50000)
        >>> buffer_pool.put_data_with_timestamp(data_array, timestamp_us=987654321)
    """
    
    def __init__(self, daq_info: DaqInfo, data_columns: dict, size: int = 100000,  start_timestamp_us: int = 0):
        """
        Initialize the AcqBufferPool instance for buffering multi-channel data.

        Sets up the buffers for each channel as defined in the provided `DaqInfo` object and 
        prepares the time buffer for storing timestamps.

        Parameters:
            daq_info: An instance of `DaqInfo` to configure the channel buffers.
            data_columns: Dictionary mapping AI pins to data column indices.
            size: Number of samples in each buffer.
            start_timestamp_us: Starting timestamp offset in microseconds.
        """

        self._daq_info = deepcopy(daq_info)
        # Keep only channels which are present in the data_column dict
        for channel_name, input_info in daq_info.channel.items():
            if input_info.ai_pin not in data_columns:
                self._daq_info.channel.pop(channel_name)
        self._data_columns = data_columns
        self._buffer_size = size
        self._prepare_channel_buffer()
        self._prepare_time_buffer(start_timestamp_us)
        self.actual_sidx = -1

    def _prepare_channel_buffer(self):
        """Prepare the channel buffers based on the configuration in `daq_info`.

        Creates an `AcqBuffer` for each channel defined in `daq_info`, adjusting for 
        channel-specific delays and applying gain and offset settings.
        """
        delay_list = [channel.delay for _,channel in self._daq_info.channel.items()]
        max_delay = max(delay_list)
        self.channel = {}
        for channel_name, channel_info in self._daq_info.channel.items():
            self.channel[channel_name] = AcqBuffer(size=self._buffer_size, 
                                                   scale_gain=channel_info.gain, 
                                                   scale_offset=channel_info.offset, 
                                                   sample_delay=max_delay-channel_info.delay,
                                                   name=channel_name)
    
    def _prepare_time_buffer(self, start_timestamp: int):
        """Prepare the time buffer for storing timestamps.

        Initializes the time buffer and sets the starting timestamp. The time buffer is used 
        to keep track of the timestamps associated with the acquisition data.

        Parameters:
            start_timestamp: Initial timestamp offset in microseconds.
        """
        self._last_timestamp_us = start_timestamp
        self._time_batch_array = np.zeros(1)
        self.time = AcqBuffer(self._buffer_size, dtype=np.uint64)

    def put_data(self, data: np.array):
        """Add data to the channel buffers.

        Distributes the provided data across the buffers for each channel. The number of columns 
        in `data` must match the number of channels defined in `daq_info`.

        Parameters:
            data: A 2D numpy array where each column corresponds to a channel's data.
        """
        for channel_name, input_info in self._daq_info.channel.items():
            self.channel[channel_name].put_data(data[:,self._data_columns[input_info.ai_pin]])
        self.actual_sidx += data.shape[0]

    def add_timestamp(self, timestamp_us: int, num_samples: int):
        """Add timestamps to the time buffer.

        Generates a series of timestamps based on the provided `timestamp_us` and `num_samples`, 
        and stores them in the time buffer.

        Parameters:
            timestamp_us: Timestamp of the most recent sample in microseconds.
            num_samples: Number of samples to generate timestamps for.
        """
        if num_samples != self._time_batch_array.shape[0]:
            self._time_batch_array = np.arange(0, num_samples) # TODO: Check if 1 must be added or not
        self.time.put_data(self._time_batch_array*(timestamp_us - self._last_timestamp_us)/num_samples+self._last_timestamp_us)
        self._last_timestamp_us = timestamp_us

    def put_data_with_timestamp(self, data: np.array, timestamp_us: int):
        """Add channel data to the buffers along with the corresponding timestamp.

        Adds the provided data to the channel buffers and updates the time buffer with 
        the given `timestamp_us`.

        Parameters:
            data: A 2D numpy array where each column corresponds to a channel's data.
            timestamp_us: Timestamp of the most recent sample in microseconds.
        """
        self.put_data(data)
        self.add_timestamp(timestamp_us, data.shape[0])

    def put_data_with_samplerate(self, data: np.array, samplerate: float):
        """Add channel data to the buffers and extend time axis with sample rate.

        Adds the provided data to the channel buffers and updates the time buffer with 
        sample converted to a timestamp.

        Parameters:
            data: A 2D numpy array where each column corresponds to a channel's data.
            samplerate: Samplerate of the data acquired.
        """
        self.put_data(data)
        timestamp_us = self._last_timestamp_us + data.shape[0]*1e6/samplerate
        self.add_timestamp(timestamp_us, data.shape[0])

    def reset(self):
        """Reset all underlying buffer, clearing all stored data.

        Resets the buffer to its initial state by zeroing out all data and resetting 
        the write index and sample count.
        """
        for channel_name, channel_buf in self.channel.items():
            channel_buf.reset()
        self.time.reset()
        self._last_timestamp_us = 0
        self._time_batch_array = np.zeros(1)


class AcqBuffer(object):
    def __init__(self, size: int=100000, scale_gain: float = 1.0, scale_offset: float = 0.0, sample_delay: int = 0, dtype: np.dtype = np.float32, name: str=None):
        """Initialize the AcqBuffer instance for buffering acquired data in a cyclic manner.

        Sets up a cyclic buffer with the specified size, data type, scaling, and offset 
        properties. Optionally, a sample delay can be specified to adjust data reads.

        Parameters:
            size: Size of the buffer (number of elements).
            scale_gain: Gain applied to the data upon storage.
            scale_offset: Offset applied to the data upon storage.
            sample_delay: Delay applied to the data during reads.
            dtype: Datatype of the buffer.
            name: Optional name for the buffer instance.
        """
        self._data = np.zeros(size, dtype=dtype)
        self.last_write_idx = 0
        self.sample_count = 0
        self.scale_gain = scale_gain
        self.scale_offset = scale_offset
        self.sample_delay = sample_delay
        self.last_sample_value = 0
        self.name = name
        
    def put_data(self, data: np.array) -> int:
        """Add data to the buffer in a cyclic manner.

        Inserts the provided data into the buffer, applying the configured gain and offset. 
        If the data size exceeds the available space, the buffer wraps around and overwrites 
        the oldest data.

        Parameters:
            data: A numpy array of data to be added to the buffer.
        
        Returns:
            The updated total sample count in the buffer.
        """
        # Split data into two parts if remaining buffer size is smaller than data
        if self.last_write_idx+len(data) > len(self._data):
            buffer_size_left = len(self._data) - self.last_write_idx
            remaining_size = len(data) - buffer_size_left
            self._data[self.last_write_idx:] = data[:buffer_size_left]*self.scale_gain - self.scale_offset
            self._data[:remaining_size] = data[buffer_size_left:]*self.scale_gain - self.scale_offset
            self.last_write_idx = remaining_size
        else:
            self._data[self.last_write_idx:self.last_write_idx+len(data)] = data*self.scale_gain - self.scale_offset
            self.last_write_idx += len(data)
        self.sample_count += len(data)
        self.last_sample_value = self._data[self.last_write_idx-1]
        return self.sample_count
        
    def read_data_by_index(self, start_idx: int, stop_idx: int) -> np.ndarray:
        """Read data from the buffer by specifying a sample index range.

        Retrieves data from the buffer starting from `start_idx` up to `stop_idx`, 
        applying the sample delay if configured. The data read may overlap if the indices 
        wrap around the cyclic buffer.

        Parameters:
            start_idx: Starting sample index (inclusive).
            stop_idx: Stopping sample index (exclusive).
        
        Returns:
            An array containing the requested data range. Returns `None` if the indices are out of bounds.
        """
        start_idx -= self.sample_delay
        stop_idx -= self.sample_delay
        if start_idx > self.sample_count or stop_idx > self.sample_count:
            return None
        start_idx %= len(self._data)
        stop_idx %= len(self._data)
        # Return overlapping data
        if stop_idx < start_idx:
            data = np.r_[self._data[start_idx:], self._data[:stop_idx]]
            return data
        # Return non overlapping data
        else:
            return self._data[start_idx:stop_idx]

    def reset(self):
        """Reset the buffer, clearing all stored data.

        Resets the buffer to its initial state by zeroing out all data and resetting 
        the write index and sample count.
        """
        self._data *= 0
        self.last_write_idx = 0
        self.sample_count = 0
        self.last_sample_value = 0