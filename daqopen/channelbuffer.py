"""
Module: channelbuffer
Description: module for buffering acquisition and channel data

Author: Michael Oberhofer
Created on: 2023-12-10
Last Updated: 2023-12-10

License: MIT

Notes: 

Version: 0.01
Github: https://github.com/DaqOpen/daqopen-lib/daqopen/
"""
import numpy as np
from modules.daqinfo import DaqInfo

class AcqBufferPool(object):
    def __init__(self, daq_info: DaqInfo, size: int = 100000, start_timestamp_us: int = 0):
        """Create AcqBufferPool object for convinient use with more than 1 channel and a single timebase

        Args:
            daq_info (DaqInfo): DaqInfo object from which to create the buffer
            size (int, optional): Number of samples in the buffer. Defaults to 100000.
            start_timestamp_us (int, optional): Optional start offset of time buffer. Defaults to 0.
        """
        self._daq_info = daq_info
        self._buffer_size = size
        self._prepare_channel_buffer()
        self._prepare_time_buffer(start_timestamp_us)
        self.actual_sidx = -1

    def _prepare_channel_buffer(self):
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
        self._last_timestamp_us = start_timestamp
        self._time_batch_array = np.zeros(1)
        self.time = AcqBuffer(self._buffer_size, dtype=np.uint64)

    def put_data(self, data: np.array):
        """Put data into the channel buffer

        Args:
            data (np.array): samples to add. Number of columns must match the DaqInfo
        """
        for channel_name, channel_index in self._daq_info.channel_index.items():
            self.channel[channel_name].put_data(data[:,channel_index])
        self.actual_sidx += data.shape[0]

    def add_timestamp(self, timestamp_us: int, num_samples: int):
        """Add timestamps to the time buffer

        Args:
            timestamp_us (int): timestamp of the most recent sample in microseconds
            num_samples (int): number of samples added
        """
        if num_samples != self._time_batch_array.shape[0]:
            self._time_batch_array = np.arange(1, num_samples+1)
        self.time.put_data(self._time_batch_array*(timestamp_us - self._last_timestamp_us)/num_samples+self._last_timestamp_us)
        self._last_timestamp_us = timestamp_us

    def put_data_with_timestamp(self, data: np.array, timestamp_us: int):
        """Put channel data to buffer together with most recent timestamp

        Args:
            data (np.array): samples to add. Number of columns must match the DaqInfo
            timestamp_us (int): timestamp of the most recent sample in microseconds
        """
        self.put_data(data)
        self.add_timestamp(timestamp_us, data.shape[0])

class AcqBuffer(object):
    def __init__(self, size: int=100000, scale_gain=1.0, scale_offset=0.0, sample_delay=0 ,dtype=np.float32, name=None):
        """Create AcqBuffer object for buffering the acquired data (cyclic buffer)

        Args:
            size (int): Size of the buffer (number of elements). Defaults to 100000.
            scale_gain (float, optional): Gain scaler which will be applied to the data. Defaults to 1.0.
            scale_offset (float, optional): Offset which will be applied to the data. Defaults to 0.0.
            sample_delay (int, optional): Delay the data on read. Defaults to 0.
            dtype (_type_, optional): Datatype of the resulting np.array. Defaults to np.float32.
            name (_type_, optional): Name of the buffer instance. Defaults to None.
        """
        self._data = np.zeros(size, dtype=dtype)
        self.last_write_idx = 0
        self.sample_count = 0
        self.scale_gain = scale_gain
        self.scale_offset = scale_offset
        self.sample_delay = sample_delay
        self.last_sample_value = 0
        self.name = name
        
    def put_data(self, data: np.array):
        """ Put Data into the buffer
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
        
    def read_data_by_index(self, start_idx: int, stop_idx: int):
        """ Read data by sample index range. Include start, exclude stop
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
        self._data *= 0
        self.last_write_idx = 0
        self.sample_count = 0
        self.last_sample_value = 0