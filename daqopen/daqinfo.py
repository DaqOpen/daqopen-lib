
"""
Module: daqinfo
Description: module for defining daq-info

Author: Michael Oberhofer
Created on: 2023-11-30
Last Updated: 2023-11-30

License: MIT

Notes: 

Version: 0.01
Github: https://github.com/DaqOpen/daqopen-lib/daqopen/
"""

from dataclasses import dataclass
from typing import List, Dict
import struct

@dataclass
class InputInfo:
    gain: float = 1.0
    offset: float = 0.0
    delay: int = 0
    unit: str = "V"
    ad_index: int = -1

class DaqInfo(object):
    def __init__(self, samplerate: float, channel_info: Dict[str, InputInfo]):
        self.samplerate = samplerate
        self.channel_index = {}
        self.channel_name = {}
        for ch_name, ch_info in channel_info.items():
            self.channel_index[ch_name] = ch_info.ad_index
            self.channel_name[ch_info.ad_index] = ch_name
        self.channel = channel_info

    @classmethod
    def from_dict(cls, data):
        channel_info = {}
        channel_index = {}
        for ch_name, ch_info in data["channel"].items():
            channel_info[ch_name] = InputInfo(gain=ch_info["gain"], offset=ch_info["offset"], delay=ch_info["delay"], unit=ch_info["unit"], ad_index = ch_info["ad_index"])
        return cls(samplerate=data["samplerate"], channel_info=channel_info)

    @classmethod
    def from_binary(cls, data):
        channel_info = {}
        samplerate = struct.unpack_from("d", data, 0)[0]
        ch_data_struct = struct.Struct("4sffb4s")
        num_channels = (len(data) - 8) // ch_data_struct.size
        for idx in range(num_channels):
            name, gain, offset, delay, unit = ch_data_struct.unpack_from(data, 8+idx*ch_data_struct.size)
            channel_info[name.decode().replace("\x00","")] = InputInfo(gain=gain, offset=offset, delay=delay, unit=unit.decode().replace("\x00",""), ad_index = idx)
        return cls(samplerate=samplerate, channel_info=channel_info)

    def to_dict(self):
        channel_info = {}
        for ch_name, ch_info in self.channel.items():
            channel_info[ch_name] = ch_info.__dict__
        return {"samplerate": self.samplerate, "channel": channel_info}

    def apply_sensor_to_channel(self, ch_name, sensor_info: InputInfo):
        self.channel[ch_name].gain *= sensor_info.gain
        self.channel[ch_name].offset *= sensor_info.gain
        self.channel[ch_name].offset += sensor_info.offset
        self.channel[ch_name].delay += sensor_info.delay
        self.channel[ch_name].unit = sensor_info.unit

    def to_binary(self):
        binary_data = struct.pack("d", self.samplerate)
        for idx in range(max(self.channel_index.values())+1):
            if idx in self.channel_name:
                ch_name = self.channel_name[idx]
                gain = self.channel[ch_name].gain
                offset = self.channel[ch_name].offset
                delay = self.channel[ch_name].delay
                unit = self.channel[ch_name].unit
                channel_data = struct.pack("4sffb4s", ch_name.encode(), gain, offset, delay, unit.encode())
            else:
                channel_data = struct.pack("4sffb4s", b"", 1.0, 0.0, 0, b"")
            binary_data += channel_data
        return binary_data

    def __str__(self):
        return f"{self.__class__.__name__}(samplerate={self.samplerate})"


if __name__ == "__main__":

    info_dict = {"samplerate": 48000,
                 "channel": {"U1": {"gain": 1.0, "offset": 1.0, "delay": 1, "unit": "V", "ad_index": 0},
                             "U2": {"gain": 2.0, "offset": 2.0, "delay": 2, "unit": "V", "ad_index": 1}}}
    myDaqInfo = DaqInfo.from_dict(info_dict)
    myDaqInfo.apply_sensor_to_channel("U1", InputInfo(2, 1, 0))
    print(myDaqInfo.to_dict())
    binary_repr = myDaqInfo.to_binary()
    print(len(binary_repr))
    myNewDaqInfo = DaqInfo.from_binary(binary_repr)
    print(myNewDaqInfo.to_dict())