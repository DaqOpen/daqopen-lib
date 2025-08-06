
# daqopen/daqinfo.py

"""Module for defining data acquisition (DAQ) information.

This module provides classes to represent and manipulate the configuration information 
for data acquisition systems. The primary classes are `DaqInfo`, which encapsulates 
the DAQ system's configuration, and `InputInfo`, which holds detailed information about 
each input channel, along with `BoardInfo` to define board-level properties.

## Usage

The `DaqInfo` class serves as the main interface for managing DAQ configuration, including 
loading from and saving to different formats such as dictionaries and binary data. 
The `InputInfo` class defines the attributes of individual input channels, such as gain, offset, 
delay, and unit, while `BoardInfo` captures system-level settings such as sample rate and board type.

Examples:
    Creating a `DaqInfo` instance from a dictionary:

    >>> info_dict = {
    >>>     "board": {
    >>>         "samplerate": 48000,
    >>>         "type": "default"
    >>>     },
    >>>     "channel": {
    >>>         "U1": {"gain": 1.0, "offset": 1.0, "delay": 1, "unit": "V", "ai_pin": "A0"},
    >>>         "U2": {"gain": 2.0, "offset": 2.0, "delay": 2, "unit": "V", "ai_pin": "A1"}
    >>>     }
    >>> }
    >>> myDaqInfo = DaqInfo.from_dict(info_dict)

Classes:
    DaqInfo: Represents the configuration of the DAQ system.
    InputInfo: Defines the properties of an input channel.
    BoardInfo: Defines the properties of the DAQ board.

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
        gain (float): The gain applied to the input channel.
        offset (float): The offset applied to the input channel.
        delay (int): The delay in sample periods for this channel.
        unit (str): The unit of the measurement.
        ai_pin (str): The analog input pin name (e.g., "A0").
        sensor (str): Name of sensor used

    Examples:
        >>> input_info = InputInfo(gain=2.0, offset=1.0, delay=5, unit="V", ai_pin="A0")
    """
    gain: float = 1.0
    offset: float = 0.0
    delay: int = 0
    unit: str = "V"
    ai_pin: str = ""
    sensor: str = ""

@dataclass
class BoardInfo:
    """Represents the configuration of the DAQ board.

    `BoardInfo` stores board-level settings, such as the sample rate, board type, and whether 
    the configuration is differential or single-ended. It also allows configuration of the 
    gain and offset mode settings.

    Attributes:
        type (str): The type of the board.
        samplerate (float): The sampling rate in Hz.
        differential (bool): Specifies if the configuration is differential (default: False).
        gain (str): Gain setting for the board (default: "SGL_1X").
        offset_enabled (bool): Specifies if offset mode is enabled (default: False).
        adc_range (list): Range of the ADC [min, max] to calculate the physical range (default: [0, 4095])
        adc_clock_gain (float): Adjustment factor for adc clock (default: 1.0)
        adc_delay_seconds (float): Time delay between physical input and reading data in application

    Examples:
        >>> board_info = BoardInfo(type="duedaq", samplerate=50000)
    """
    type: str
    samplerate: float
    differential: bool = False
    gain: str = "SGL_1X"
    offset_enabled: bool = False
    adc_range: tuple = (0, 4095)
    adc_clock_gain: float = 1.0
    adc_delay_seconds: float = 0.0

class DaqInfo(object):
    """Represents the configuration of the data acquisition (DAQ) system.

    `DaqInfo` contains information about the DAQ system's sampling rate and the configuration 
    of each input channel. It provides methods for creating an instance from various formats 
    (e.g., dictionary, binary data) and for applying sensor adjustments to channels.

    Attributes:
        board (BoardInfo): The board-level information of the DAQ system.
        channel (dict): A dictionary of `InputInfo` objects, keyed by channel name.
        ai_pin_name (dict): Maps channel names to their analog-to-digital (AD) indices.
        channel_name (dict): Maps AD indices to channel names.

    Methods:
        from_dict(data): Class method to create a `DaqInfo` instance from a dictionary.
        get_default(): Class method to create a default `DaqInfo` instance.
        to_dict(): Converts the `DaqInfo` instance into a dictionary format.
        apply_sensor_to_channel(ch_name, sensor_info): Applies sensor configuration to a specific channel.
        __str__(): Returns a string representation of the `DaqInfo` instance.

    Examples:
        >>> info_dict = {
        >>>     "board": {
        >>>         "samplerate": 48000,
        >>>         "type": "default"
        >>>     },
        >>>     "channel": {
        >>>         "U1": {"gain": 1.0, "offset": 1.0, "delay": 1, "unit": "V", "ai_pin": "A0"},
        >>>         "U2": {"gain": 2.0, "offset": 2.0, "delay": 2, "unit": "V", "ai_pin": "A1"}
        >>>     }
        >>> }
        >>> myDaqInfo = DaqInfo.from_dict(info_dict)
    """
    def __init__(self, board_info: BoardInfo, channel_info: Dict[str, InputInfo], sensor_info: Dict[str, InputInfo] = {}):
        """Initialize the DaqInfo instance with the specified board and channel information.

        Sets up the DAQ configuration, mapping channel names to their analog-to-digital (AD) indices 
        and vice versa. Stores the input channel configurations provided in `channel_info`.

        Parameters:
            board_info (BoardInfo): The board information as an instance of `BoardInfo`.
            channel_info (dict): A dictionary mapping channel names to `InputInfo` instances.

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
                ch_info.ai_pin = ch_name
            self.channel_name[self.ai_pin_name[ch_name]] = ch_name
        self.channel = channel_info
        self.sensor = sensor_info

    @classmethod
    def from_dict(cls, data: dict):
        """Create a DaqInfo instance from a dictionary.

        Converts a dictionary containing DAQ configuration information into a `DaqInfo` instance. 
        The dictionary should include a `board` key with board information, and a `channel` key 
        that maps channel names to their configurations.

        Parameters:
            data (dict): A dictionary containing DAQ configuration data.

        Returns:
            DaqInfo: A new instance of `DaqInfo` populated with the provided data.

        Notes:
            Expected format:
                {
                    "board": {
                        "samplerate": float,
                        "type": str
                    },
                    "channel": {
                        "ChannelName": {
                            "gain": float,
                            "offset": float,
                            "delay": int,
                            "unit": str,
                            "ai_pin": str
                        },
                        ...
                    }
                }

        """
        board_info = BoardInfo(**data["board"])
        channel_info = {}
        sensor_info = {}
        for ch_name, ch_info in data["channel"].items():
            if not ch_info.get("enabled", True):
                continue
            channel_info[ch_name] = InputInfo(gain=ch_info.get("gain", 1.0), 
                                              offset=ch_info.get("offset", 0.0), 
                                              delay=ch_info.get("delay", 0), 
                                              unit=ch_info.get("unit","V"), 
                                              ai_pin = ch_info.get("ai_pin",""),
                                              sensor=ch_info.get("sensor",""))
        if "sensor" in data:
            for sensor_name, sensor in data["sensor"].items():
                sensor_info[sensor_name] = InputInfo(gain=sensor.get("gain", 1.0), 
                                                offset=sensor.get("offset", 0.0),
                                                delay=sensor.get("delay", 0), 
                                                unit=sensor.get("unit","V"))
        return cls(board_info=board_info, channel_info=channel_info, sensor_info=sensor_info)

    @classmethod
    def get_default(cls):
        """Create a default DaqInfo Object

        Returns a `DaqInfo` instance with default board and channel configurations.

        Returns:
            DaqInfo: A new `DaqInfo` instance with default values.
        """
        board_info = BoardInfo(type="default", samplerate=0)
        channel_info = {}
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
        sensor_info = {}
        if self.sensor:
            for sensor_name, sensor in self.sensor.items():
                sensor_info[sensor_name] = sensor.__dict__
        return {"board": self.board.__dict__, "channel": channel_info, "sensor": sensor_info}

    def _apply_sensor_to_channel(self, channel_info: InputInfo, sensor_info: InputInfo) -> InputInfo:
        """Apply sensor configuration to a specific channel.

        Merges the gain, offset, and delay of the specified channel based on the provided 
        sensor information. The sensor's configuration is combined with the existing channel 
        configuration.

        Parameters:
            channel_info: An `InputInfo` instance containing the channels configuration.
            sensor_info: An `InputInfo` instance containing the sensor's configuration.

        Returns:
            InputInfo representing the combination of channel and sensor

        Examples:
            >>> channel_info = InputInfo(gain=1.0)
            >>> sensor_info = InputInfo(gain=2.0, offset=1.0, delay=0)
            >>> daq_info.apply_sensor_to_channel(channel_info, sensor_info)
        """
        result_channel = InputInfo()
        result_channel.gain = channel_info.gain * sensor_info.gain
        result_channel.offset = channel_info.offset * sensor_info.gain
        result_channel.offset += sensor_info.offset
        result_channel.delay = channel_info.delay + sensor_info.delay
        result_channel.unit = sensor_info.unit
        return result_channel

    def get_channel_info_with_sensor(self):
        """Apply sensors to channels

        Apply the sensor data to the channel's InputInfo.

        Raises:
            ValueError: If sensor can't be found
        """
        channel_info_with_sensor = {}
        for ch_name, ch_info in self.channel.items():
            if ch_info.sensor:
                try:
                    channel_info_with_sensor[ch_name] = self._apply_sensor_to_channel(ch_info, self.sensor[ch_info.sensor])
                except KeyError:
                    raise ValueError("Sensor not found:", ch_info.sensor)
            else:
                channel_info_with_sensor[ch_name] = ch_info
        return channel_info_with_sensor

    def __str__(self) -> str:
        """Return a string representation of the DaqInfo instance.

        Provides a concise string summary of the DAQ configuration, primarily showing the 
        sampling rate.

        Returns:
            A string describing the `DaqInfo` instance.

        Examples:
            >>> daq_info = DaqInfo(...)
            >>> print(str(daq_info))
            DaqInfo(type=duedaq,samplerate=48000)
        """
        return f"{self.__class__.__name__}(type={self.board.type},samplerate={self.board.samplerate})"    


if __name__ == "__main__":

    info_dict = {"samplerate": 48000,
                 "channel": {"U1": {"gain": 1.0, "offset": 1.0, "delay": 1, "unit": "V", "ai_pin": "A0"},
                             "U2": {"gain": 2.0, "offset": 2.0, "delay": 2, "unit": "V", "ai_pin": "A1"}}}
    myDaqInfo = DaqInfo.from_dict(info_dict)
    myDaqInfo._apply_sensor_to_channel("U1", InputInfo(2, 1, 0))
    print(myDaqInfo.to_dict())
