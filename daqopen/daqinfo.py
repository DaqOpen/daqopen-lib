
# daqopen/daqinfo.py

"""Module for defining data acquisition (DAQ) information.

This module provides classes to represent and manipulate the configuration information 
for data acquisition systems. The primary classes are `DaqInfo`, which encapsulates 
the DAQ system's configuration, and `InputInfo`, which holds detailed information about 
each input channel.

## Usage

The `DaqInfo` class serves as the main interface for managing DAQ configuration, including 
loading from and saving to different formats such as dictionaries and binary data. 
The `InputInfo` class defines the attributes of individual input channels, such as gain, offset, 
delay, and unit.

Examples:
    Creating a `DaqInfo` instance from a dictionary:

    >>> info_dict = {
    >>>     "samplerate": 48000,
    >>>     "channel": {
    >>>         "U1": {"gain": 1.0, "offset": 1.0, "delay": 1, "unit": "V", "ad_index": 0},
    >>>         "U2": {"gain": 2.0, "offset": 2.0, "delay": 2, "unit": "V", "ad_index": 1}
    >>>     }
    >>> }
    >>> myDaqInfo = DaqInfo.from_dict(info_dict)

Classes:
    DaqInfo: Represents the configuration of the DAQ system.
    InputInfo: Defines the properties of an input channel.

"""

from dataclasses import dataclass
from typing import List, Dict
import struct

@dataclass
class InputInfo:
    """Represents the configuration of a single input channel.

    `InputInfo` stores the properties of an individual input channel, including the gain, 
    offset, delay, unit, and analog-to-digital (AD) index. This class is used to encapsulate 
    the settings for each channel in a DAQ system.

    Attributes:
        gain: The gain applied to the input channel.
        offset: The offset applied to the input channel.
        delay: The delay in sample periods for this channel.
        unit: The unit of the measurement.
        ad_index: The index of the analog-to-digital converter channel.

    Examples:
        >>> input_info = InputInfo(gain=2.0, offset=1.0, delay=5, unit="V", ad_index=0)
    """
    gain: float = 1.0
    offset: float = 0.0
    delay: int = 0
    unit: str = "V"
    ad_index: int = -1

class DaqInfo(object):
    """Represents the configuration of the data acquisition (DAQ) system.

    `DaqInfo` contains information about the DAQ system's sampling rate and the configuration 
    of each input channel. It provides methods for creating an instance from various formats 
    (e.g., dictionary, binary data) and for applying sensor adjustments to channels.

    Attributes:
        samplerate: The sampling rate of the DAQ system in Hz.
        channel: A dictionary of `InputInfo` objects, keyed by channel name.
        channel_index: Maps channel names to their analog-to-digital (AD) indices.
        channel_name: Maps AD indices to channel names.

    Methods:
        from_dict(data): Class method to create a `DaqInfo` instance from a dictionary.
        from_binary(data): Class method to create a `DaqInfo` instance from binary data.
        to_dict(): Converts the `DaqInfo` instance into a dictionary format.
        apply_sensor_to_channel(ch_name, sensor_info): Applies sensor configuration to a specific channel.
        to_binary(): Converts the `DaqInfo` instance into a binary format.

    Examples:
        >>> info_dict = {
        >>>     "samplerate": 48000,
        >>>     "channel": {
        >>>         "U1": {"gain": 1.0, "offset": 1.0, "delay": 1, "unit": "V", "ad_index": 0},
        >>>         "U2": {"gain": 2.0, "offset": 2.0, "delay": 2, "unit": "V", "ad_index": 1}
        >>>     }
        >>> }
        >>> myDaqInfo = DaqInfo.from_dict(info_dict)
    """
    def __init__(self, samplerate: float, channel_info: Dict[str, InputInfo]):
        """Initialize the DaqInfo instance with the specified sampling rate and channel information.

        Sets up the DAQ configuration, mapping channel names to their analog-to-digital (AD) indices 
        and vice versa. Stores the input channel configurations provided in `channel_info`.

        Parameters:
            samplerate: The sampling rate of the DAQ system in Hz.
            channel_info: A dictionary mapping channel names to `InputInfo` instances.

        Examples:
            >>> channel_info = {
            >>>     "U1": InputInfo(gain=1.0, offset=1.0, delay=1, unit="V", ad_index=0),
            >>>     "U2": InputInfo(gain=2.0, offset=2.0, delay=2, unit="V", ad_index=1)
            >>> }
            >>> daq_info = DaqInfo(samplerate=48000, channel_info=channel_info)
        """
        self.samplerate = samplerate
        self.channel_index = {}
        self.channel_name = {}
        for ch_name, ch_info in channel_info.items():
            self.channel_index[ch_name] = ch_info.ad_index
            self.channel_name[ch_info.ad_index] = ch_name
        self.channel = channel_info

    @classmethod
    def from_dict(cls, data: dict):
        """Create a DaqInfo instance from a dictionary.

        Converts a dictionary containing DAQ configuration information into a `DaqInfo` instance. 
        The dictionary should include a `samplerate` key and a `channel` key that maps channel names 
        to their configurations.

        Parameters:
            data: A dictionary containing DAQ configuration data.

        Notes:
            Expected format:
                {
                    "samplerate": float,
                    "channel": {
                        "ChannelName": {
                            "gain": float,
                            "offset": float,
                            "delay": int,
                            "unit": str,
                            "ad_index": int
                        },
                        ...
                    }
                }

        """
        channel_info = {}
        channel_index = {}
        for ch_name, ch_info in data["channel"].items():
            channel_info[ch_name] = InputInfo(gain=ch_info["gain"], offset=ch_info["offset"], delay=ch_info["delay"], unit=ch_info["unit"], ad_index = ch_info["ad_index"])
        return cls(samplerate=data["samplerate"], channel_info=channel_info)

    @classmethod
    def from_binary(cls, data: bytes):
        """Create a DaqInfo instance from binary data.

    Parses binary data to extract DAQ configuration information, including the sampling rate 
    and input channel configurations. The binary format is defined by a specific structure, 
    with each channel described by a fixed set of fields.

    Parameters:
        data: Binary data containing DAQ configuration.
    """
        channel_info = {}
        samplerate = struct.unpack_from("d", data, 0)[0]
        ch_data_struct = struct.Struct("4sffb4s")
        num_channels = (len(data) - 8) // ch_data_struct.size
        for idx in range(num_channels):
            name, gain, offset, delay, unit = ch_data_struct.unpack_from(data, 8+idx*ch_data_struct.size)
            channel_info[name.decode().replace("\x00","")] = InputInfo(gain=gain, offset=offset, delay=delay, unit=unit.decode().replace("\x00",""), ad_index = idx)
        return cls(samplerate=samplerate, channel_info=channel_info)

    def to_dict(self) -> dict:
        """Convert the DaqInfo instance into a dictionary.

        Serializes the DAQ configuration into a dictionary format, suitable for storage or 
        further processing.

        Returns:
            A dictionary representation of the `DaqInfo` instance.
        """
        channel_info = {}
        for ch_name, ch_info in self.channel.items():
            channel_info[ch_name] = ch_info.__dict__
        return {"samplerate": self.samplerate, "channel": channel_info}

    def apply_sensor_to_channel(self, ch_name: str, sensor_info: InputInfo):
        """Apply sensor configuration to a specific channel.

        Adjusts the gain, offset, and delay of the specified channel based on the provided 
        sensor information. The sensor's configuration is combined with the existing channel 
        configuration.

        Parameters:
            ch_name: The name of the channel to which the sensor configuration is applied.
            sensor_info: An `InputInfo` instance containing the sensor's configuration.

        Examples:
            >>> sensor_info = InputInfo(gain=2.0, offset=1.0, delay=0)
            >>> daq_info.apply_sensor_to_channel("U1", sensor_info)
        """
        self.channel[ch_name].gain *= sensor_info.gain
        self.channel[ch_name].offset *= sensor_info.gain
        self.channel[ch_name].offset += sensor_info.offset
        self.channel[ch_name].delay += sensor_info.delay
        self.channel[ch_name].unit = sensor_info.unit

    def to_binary(self) -> bytes:
        """Convert the DaqInfo instance into binary format.

        Serializes the DAQ configuration, including the sampling rate and channel information, 
        into a binary format. This binary representation can be used for compact storage or 
        transmission.

        Returns:
            Binary data representing the `DaqInfo` instance.

        Examples:
            >>> daq_info = DaqInfo(...)
            >>> binary_data = daq_info.to_binary()
        """
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

    def __str__(self) -> str:
        """Return a string representation of the DaqInfo instance.

        Provides a concise string summary of the DAQ configuration, primarily showing the 
        sampling rate.

        Returns:
            A string describing the `DaqInfo` instance.

        Examples:
            >>> daq_info = DaqInfo(...)
            >>> print(str(daq_info))
            DaqInfo(samplerate=48000)
        """
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