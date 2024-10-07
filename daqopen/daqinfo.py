
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
    >>>         "U1": {"gain": 1.0, "offset": 1.0, "delay": 1, "unit": "V", "ai_pin": "A0"},
    >>>         "U2": {"gain": 2.0, "offset": 2.0, "delay": 2, "unit": "V", "ai_pin": "A1"}
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
        ai_pin: The Due pin name e.g. "A0".

    Examples:
        >>> input_info = InputInfo(gain=2.0, offset=1.0, delay=5, unit="V", ai_pin="A0")
    """
    gain: float = 1.0
    offset: float = 0.0
    delay: int = 0
    unit: str = "V"
    ai_pin: str = ""

@dataclass
class BoardInfo:
    """ Represents the configuration of board properties
    """
    samplerate: float
    differential: bool = False
    gain: str = "SGL_1X"
    offset_enabled: bool = False

class DaqInfo(object):
    """Represents the configuration of the data acquisition (DAQ) system.

    `DaqInfo` contains information about the DAQ system's sampling rate and the configuration 
    of each input channel. It provides methods for creating an instance from various formats 
    (e.g., dictionary, binary data) and for applying sensor adjustments to channels.

    Attributes:
        samplerate: The sampling rate of the DAQ system in Hz.
        channel: A dictionary of `InputInfo` objects, keyed by channel name.
        ai_pin_name: Maps channel names to their analog-to-digital (AD) indices.
        channel_name: Maps AD indices to channel names.

    Methods:
        from_dict(data): Class method to create a `DaqInfo` instance from a dictionary.
        from_binary(data): Class method to create a `DaqInfo` instance from binary data.
        to_dict(): Converts the `DaqInfo` instance into a dictionary format.
        apply_sensor_to_channel(ch_name, sensor_info): Applies sensor configuration to a specific channel.
        to_binary(): Converts the `DaqInfo` instance into a binary format.

    Examples:
        >>> info_dict = {
        >>>     "board": {
        >>>         "samplerate": 48000
        >>>     },
        >>>     "channel": {
        >>>         "U1": {"gain": 1.0, "offset": 1.0, "delay": 1, "unit": "V", "ai_pin": "A0"},
        >>>         "U2": {"gain": 2.0, "offset": 2.0, "delay": 2, "unit": "V", "ai_pin": "A1"}
        >>>     }
        >>> }
        >>> myDaqInfo = DaqInfo.from_dict(info_dict)
    """
    def __init__(self, board_info: BoardInfo, channel_info: Dict[str, InputInfo]):
        """Initialize the DaqInfo instance with the specified sampling rate and channel information.

        Sets up the DAQ configuration, mapping channel names to their analog-to-digital (AD) indices 
        and vice versa. Stores the input channel configurations provided in `channel_info`.

        Parameters:
            samplerate: The sampling rate of the DAQ system in Hz.
            channel_info: A dictionary mapping channel names to `InputInfo` instances.

        Examples:
            >>> channel_info = {
            >>>     "U1": InputInfo(gain=1.0, offset=1.0, delay=1, unit="V", ai_pin="A0"),
            >>>     "U2": InputInfo(gain=2.0, offset=2.0, delay=2, unit="V", ai_pin="A1")
            >>> }
            >>> board_info = BoardInfo(samplerate=50000)
            >>> daq_info = DaqInfo(board_info=board_info, channel_info=channel_info)
        """
        self.board = board_info
        self.ai_pin_name = {}
        self.channel_name = {}
        for ch_name, ch_info in channel_info.items():
            if ch_info.ai_pin:
                self.ai_pin_name[ch_name] = ch_info.ai_pin
            else:
                self.ai_pin_name[ch_name] = ch_name
            self.channel_name[self.ai_pin_name[ch_name]] = ch_name
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
                    "board" : {
                        "samplerate": float
                    }
                    "channel": {
                        "ChannelName": {
                            "gain": float,
                            "offset": float,
                            "delay": int,
                            "unit": str,
                            "ai_pin": int
                        },
                        ...
                    }
                }

        """
        board_info = BoardInfo(**data["board"])
        channel_info = {}
        for ch_name, ch_info in data["channel"].items():
            channel_info[ch_name] = InputInfo(gain=ch_info["gain"], offset=ch_info["offset"], delay=ch_info["delay"], unit=ch_info["unit"], ai_pin = ch_info["ai_pin"])
        return cls(board_info=board_info, channel_info=channel_info)

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
        return {"board": self.board.__dict__, "channel": channel_info}

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
        return f"{self.__class__.__name__}(samplerate={self.board.samplerate})"    

if __name__ == "__main__":

    info_dict = {"samplerate": 48000,
                 "channel": {"U1": {"gain": 1.0, "offset": 1.0, "delay": 1, "unit": "V", "ai_pin": "A0"},
                             "U2": {"gain": 2.0, "offset": 2.0, "delay": 2, "unit": "V", "ai_pin": "A1"}}}
    myDaqInfo = DaqInfo.from_dict(info_dict)
    myDaqInfo.apply_sensor_to_channel("U1", InputInfo(2, 1, 0))
    print(myDaqInfo.to_dict())
