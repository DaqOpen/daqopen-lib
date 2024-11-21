# daqopen/duedaq.py

"""Module for interacting with Arduino Due DAQ system.

This module provides classes and exceptions for data acquisition from an Arduino Due using the duq-daq firmware. 
It enables both actual data acquisition and simulation for testing purposes.

## Usage

The primary class for interacting with the DAQ system is `DueDaq`. It handles communication with the Arduino Due device, 
starts and stops data acquisition, and retrieves the data collected. For testing without hardware, `DueSerialSim` 
can simulate data acquisition by generating mock data.

Examples:
    Create a DueDaq instance with a simulated serial port

    >>> from daqopen.duedaq import DueDaq
    >>> my_daq = DueDaq(serial_port_name="SIM")
    >>> my_daq.start_acquisition()
    >>> data = my_daq.read_data()
    >>> print(data)
    [100, 100, 100]
    >>> my_daq.stop_acquisition()

Classes:
    DueSerialSim: Simulation Class for testing purpose.
    DueDaq: Driver class for actual data acquisition.

Raises:
    DeviceNotFoundException: Exception when the device could not be found by its VID and PID.
    DAQErrorException: Exception when there occurs any other error relating the data acquisition.
    AcqNotRunningException: Exception  when data will be read without acquisition running.

"""

import serial
from enum import Enum
import serial.tools.list_ports
import time
import numpy as np
import logging

logger = logging.getLogger(__name__)

class DeviceNotFoundException(Exception):
    """Exception raised when the DAQ device cannot be found by its Vendor ID (VID) and Product ID (PID).

    This exception is typically raised when attempting to initialize a connection to an Arduino Due DAQ system
    and the device is not detected on any serial port.
    """
    def __init__(self, device_name: str):
        """Exception raised when the DAQ device cannot be found by its Vendor ID (VID) and Product ID (PID)

        Parameters:
            device_name: The name or identifier of the missing device.
        """
        message = f"Device not found: {device_name}"
        super().__init__(message)

class DAQErrorException(Exception):
    """Exception raised for errors related to the DAQ system.

    This exception is raised during data acquisition if there are inconsistencies or 
    errors such as frame number mismatches, corrupted data, or unexpected behavior.
    """

    def __init__(self, error_info: str):
        """Exception raised for errors related to the DAQ system.

        Parameters:
            error_info: A description of the error encountered during data acquisition.
        """
        message = f"DAQ Error: {error_info}"
        super().__init__(message)

class AcqNotRunningException(Exception):
    """Exception raised when data is attempted to be read without an active acquisition process.

    This exception is raised if a read operation is attempted while the DAQ system is not 
    currently acquiring data. It helps prevent operations on an inactive data stream.
    """
    def __init__(self, error_info: str):
        """Exception raised when data is attempted to be read without an active acquisition process.

        Parameters:
            error_info: A description of the error encountered when attempting to read data.
        """
        message = f"Acquisition not Running Error: {error_info}"
        super().__init__(message)

class DueSerialSim(object):
    """A simulation class for Arduino Due Data Acquisition (DAQ) system.

    `DueSerialSim` is designed to simulate data acquisition from an Arduino Due for testing purposes. 
    It generates mock data packets that mimic the behavior of the real DAQ system, allowing for 
    software testing without the need for actual hardware.

    Attributes:
        MCLK: MCU Clock Frequency.
        CONV_CYCLES_PER_SAMPLE: Clock Cycles per conversion.
        MAX_BUFFER_SIZE: Maximum DMA buffer size for ADC cyclic buffer in cumulative samples.
        MIN_BUFFER_SIZE: Minimum DMA buffer size for ADC cyclic buffer in cumulative samples.
        NUM_BYTES_PER_SAMPLE: Number of bytes per data sample.
        NUM_BYTES_PKG_CNT: Number of bytes in the package counter.
        START_PATTERN: Byte pattern marking the start of a data frame.
        FRAME_NUM_DT: Data type for frame number, with little-endian byte order.
        FRAME_NUM_MAX: Maximum value for the frame number.
        ADC_RANGE: Range of the ADC values for normalization.
        CHANNEL_MAPPING: Mapping of channel names to their corresponding physical pins.
        CHANNEL_ORDER: Order of the channels in data package.

    Parameters:
        realtime: Enable or disable the real-time simulation.

    Methods:
        write: Simulates writing commands to the DAQ system (e.g., "START", "STOP", "SETMODE").
        read: Reads a specified length of data from the simulated DAQ system.
        readinto: Reads data directly into the provided buffer.
        reset_input_buffer: Resets the internal read buffer.
        
    Notes:
        Please do not use this class directly, instead use the `DueDaq` with `serial_port_name = "SIM"`
    """
    MCLK: int = 84_000_000
    CONV_CYCLES_PER_SAMPLE: int = 21
    MAX_BUFFER_SIZE: int = 20000
    MIN_BUFFER_SIZE: int = 1000
    NUM_BYTES_PER_SAMPLE: int = 2
    NUM_BYTES_PKG_CNT: int = 4
    START_PATTERN: bytearray = bytearray.fromhex('FFFF')
    FRAME_NUM_DT: np.dtype = np.dtype('uint32')
    FRAME_NUM_DT = FRAME_NUM_DT.newbyteorder('<')
    FRAME_NUM_MAX: int = np.iinfo(FRAME_NUM_DT).max
    ADC_RANGE: list = [0, 4095]
    CHANNEL_MAPPING: dict = {"A0": 7, "A1": 6, "A2": 5, "A3": 4, "A4": 3, "A5": 2,
                             "A6": 1, "A7": 0, "A8": 10, "A9": 11, "A10": 12, "A11": 13}
    CHANNEL_ORDER: list = [ai_pin for ai_pin, ai_mcu in sorted(CHANNEL_MAPPING.items(), key=lambda item: item[1])]
    
    def __init__(self, realtime: bool = True):
        """Initialize the DueSerialSim instance for simulating data acquisition.

        This constructor sets up the simulation environment for the Arduino Due DAQ system.
        It initializes the internal state, prepares the simulated data signals, and sets up the 
        delay between data packet generations if `realtime` is enabled.

        Parameters:
            realtime: Enable or disable the realtime simulation. If True, timing will simulate the 
                    behavior of the real board. Otherwise, the simulation runs as fast as possible.

        Attributes:
            response_data: Placeholder for response data, initialized as an empty byte string.
            _frame_number: Counter to keep track of the frame number.
            _actual_state: Current state of the simulator, either "started" or "stopped".
            _read_buffer: Buffer to store generated frames for reading.
            _is_differential: Boolean flag indicating whether differential mode is enabled.
            _gain_value: Gain value used in signal simulation, range 0-3.
            _offset_enabled: Boolean flag indicating whether offset is enabled in the ADC.
            _adc_prescal: ADC prescaler value, affecting the sampling rate.
            _adc_cher: ADC channel enable register, storing active channel bits.
            _channels: List of active ADC channels based on `_adc_cher`.
            _buffer_size: Size of the buffer for data samples.
            _samplerate: Calculated sample rate for the simulation.
            _samples_per_block_channel: Number of samples per block per channel.
            _signal_buffer: Buffer storing the simulated signal for all active channels.
        """
        self._realtime = realtime
        self.response_data = b""
        self._frame_number = 0
        self._actual_state = "stopped"
        self._read_buffer = b""
        # Initialitze attributes for ADC
        self._is_differential = False
        self._gain_value = 0
        self._offset_enabled = False
        self._adc_prescal = 1
        self._adc_cher = 0x0040
        self._channels = ["A1"]
        self._buffer_size = self.MAX_BUFFER_SIZE
        self._samplerate = 0
        self._samples_per_block_channel = self._buffer_size

    def _cher_to_channels(self, cher: int) -> list:
        """Convert the ADC channel enable register (cher) to a list of active channels.

        This helper function checks which bits in the `cher` register are set and maps 
        them to the corresponding channel names based on the `CHANNEL_MAPPING` attribute.

        Parameters:
            cher: Integer value representing the ADC channel enable register.

        Returns:
            list: A list of channel names corresponding to the bits set in the `cher` register.
        """
        channels = []
        for ch, bit_pos in self.CHANNEL_MAPPING.items():
            if cher & (1 << bit_pos):
                channels.append(ch)
        return channels

    def _setup_fake_adc(self):
        """Set up the fake ADC for the simulation.

        This method configures the simulation's internal ADC parameters, including sample rate 
        calculation and buffer size. It generates a sinusoidal signal for all active channels 
        with attenuation applied to subsequent channels.

        The generated signal is stored in the `_signal_buffer`, where each column represents 
        a different ADC channel and each row a sample.

        Notes:
            - The sample rate is calculated based on the MCU clock, ADC prescaler, conversion cycles, 
            and the number of active channels.
            - A sinusoidal signal is generated, with each additional channel being attenuated.
        """
        self._samplerate = int(self.MCLK/((1+self._adc_prescal)*2*self.CONV_CYCLES_PER_SAMPLE*len(self._channels)))
        self._samples_per_block_channel = self._buffer_size // len(self._channels)
        self._signal_buffer = np.zeros((self._samples_per_block_channel, len(self._channels)), dtype="int16")
        # Generate Signal
        index = np.arange(0, self._samples_per_block_channel)
        main_signal = np.clip(np.sin(2*np.pi*index/self._samples_per_block_channel) * 2048 + 2048, 0, 4095)
        #main_signal[int(self.NUM_SAMPLES/4)] = 0 # Insert Spike for testing
        for i in range(len(self._channels)):
            self._signal_buffer[:,i] = main_signal/(1.0+i) # attenuate following channels data        

    def _generate_frame(self):
        """Generate a simulated data frame and store it in the internal read buffer.

        This method creates a data frame containing the `START_PATTERN`, the current frame number, 
        and the sampled data from the `_signal_buffer`. If the `realtime` flag is enabled, 
        the method introduces a delay to simulate real-time data acquisition.

        The frame consists of:
            - START_PATTERN: Marks the beginning of a data frame.
            - Frame number: 4-byte little-endian unsigned integer.
            - Signal data: Simulated ADC samples for all active channels.

        After the frame is generated, it is stored in the `_read_buffer` for future reading.
        """
        if self._realtime:
            time.sleep(self._samples_per_block_channel/self._samplerate)
        self._frame_number += 1
        frame = self.START_PATTERN+np.array([self._frame_number], dtype=self.FRAME_NUM_DT).tobytes()
        frame += self._signal_buffer.tobytes()
        self._read_buffer = frame

    def write(self, data: bytes):
        """Simulate writing commands to the DAQ system and handle the setup accordingly.

        This method processes various commands sent to the DAQ system and adjusts the 
        internal state of the simulator based on the command.

        Parameters:
            data: A byte string containing the command to be processed.
        
        Actions:
            - The appropriate internal attributes (e.g., `_is_differential`, `_gain_value`, `_adc_prescal`) 
            are updated based on the command.
            - The method recalculates ADC parameters and regenerates the fake ADC configuration after 
            any change in settings by calling `_setup_fake_adc`.
        """
        if data == b"START\n":
            self._actual_state = "started"
            self._frame_number = 0
        elif data == b"STOP\n":
            self._actual_state = "stopped"
        elif data == b"RESET\n":
            pass
        elif b"SETMODE" in data:
            value = data.decode().split(" ")[1]
            if value == "0":
                self._is_differential = False
            elif value == "1":
                self._is_differential = True
        elif b"SETGAIN" in data:
            value = int(data.decode().split(" ")[1])
            if 0 <= value <= 3:
                self._gain_value = value
        elif b"SETOFFSET" in data:
            value = data.decode().split(" ")[1]
            if value == "0":
                self._offset_enabled = False
            elif value == "1":
                self._offset_enabled = True
        elif b"SETPRESCAL" in data:
            value = int(data.decode().split(" ")[1])
            if 1 <= value <= 255:
                self._adc_prescal = value
        elif b"SETCHANNEL" in data:
            value = int(data.decode().split(" ")[1])
            self._adc_cher = value
            self._channels = self._cher_to_channels(self._adc_cher)
        elif b"SETDMABUFFERSIZE" in data:
            value = int(data.decode().split(" ")[1])
            if self.MIN_BUFFER_SIZE <= value <= self.MAX_BUFFER_SIZE:
                self._buffer_size = value
        else:
            pass
        self._setup_fake_adc()

    def read(self, length: int = 0) -> bytes:
        """Simulate reading data from the DAQ system.

        This method retrieves a specified amount of data from the internal read buffer. 
        If the buffer is empty and the simulation is in the "started" state, a new frame is generated.

        Parameters:
            length: The number of bytes to read from the buffer. If there is not enough data, 
                    a new frame is generated if the DAQ is running.

        Returns:
            bytes: A byte string containing the requested data from the buffer. If the buffer 
                contains less data than requested, it returns whatever is available.
        """
        if len(self._read_buffer) < length and self._actual_state == "started":
            self._generate_frame()
        elif len(self._read_buffer) < length:
            data_to_send = self._read_buffer
            self._read_buffer = b""
            return data_to_send
        data_to_send = self._read_buffer[:length]
        self._read_buffer = self._read_buffer[length:]
        return data_to_send

    def readinto(self, buffer: bytearray = 0):
        """Simulate reading data into an existing buffer.

        This method fills the provided buffer with data from the internal read buffer. If the simulation 
        is in the "started" state, a new data frame is generated. If data already exists in the buffer 
        when a new frame is generated, a warning is logged.

        Parameters:
            buffer: A bytearray that will be filled with the simulated data.
        
        Actions:
            - The internal read buffer is filled with the generated frame.
            - The provided buffer is then populated with this data.
        """
        if self._actual_state == "started":
            if self._read_buffer:
                logger.warning(f"Warning - Buffer not empty before new fillup: {len(self._read_buffer)}")
            self._generate_frame()
            buffer[:] = self._read_buffer[:]
            self._read_buffer = b""
    
    def reset_input_buffer(self):
        """Reset the internal read buffer of the simulator.

        This method clears the current content of the `_read_buffer`, ensuring that no 
        previously generated frames remain in the buffer.

        It can be used to reset the state when switching between different commands or tests.
        """
        self._read_buffer = b""

class DueDaqGain(Enum):
    """ Enumeration for GAIN Setting

    Attributes:
        SGL_1X: Single Ended Mode Gain = 1x
        SGL_2X: Single Ended Mode Gain = 2x
        SGL_4X: Single Ended Mode Gain = 4x
        DIFF_05X: Differential Mode Gain = 0.5x
        DIFF_1X: Differential Mode Gain = 1x
        DIFF_2X: Differential Mode Gain = 2x
    """
    SGL_1X: int = 0x01
    SGL_2X: int = 0x02
    SGL_4X: int = 0x03
    DIFF_05X: int = 0x00
    DIFF_1X: int = 0x01
    DIFF_2X: int = 0x02

class DueDaq(object):
    """
    Driver class for data acquisition from the Arduino Due DAQ system.

    The `DueDaq` class interfaces with the Arduino Due running the duq-daq firmware. 
    It handles starting and stopping data acquisition, reading and processing the 
    data collected, and managing communication over the serial interface. Additionally, 
    it supports simulated data acquisition for testing purposes.

    Attributes:
        MCLK: MCU Clock Frequency.
        CONV_CYCLES_PER_SAMPLE: Clock cycles per ADC conversion.
        ADC_PRESCAL_MAX: Limit of ADC prescaler value
        MAX_BUFFER_SIZE: Maximum DMA buffer size for ADC cyclic buffer in cumulative samples.
        MIN_BUFFER_SIZE: Minimum DMA buffer size for ADC cyclic buffer in cumulative samples.
        MAX_BUFFER_DURATION: Maximum time duration of the buffer for responsiveness.
        NUM_BYTES_PER_SAMPLE: Number of bytes per data sample.
        NUM_BYTES_PKG_CNT: Number of bytes for the package counter.
        START_PATTERN: Byte pattern marking the start of a data frame.
        FRAME_NUM_DT: Data type for frame number, with little-endian byte order.
        FRAME_NUM_MAX: Maximum value for the frame number.
        ADC_RANGE: Range of the ADC values for normalization.
        CHANNEL_MAPPING: Mapping of channel names to their corresponding physical pins.
        CHANNEL_ORDER: Order of the channels in the data package.

    Parameters:
        channels: List of channels to be acquired.
        reset_pin: GPIO pin number for hardware reset (default: None).
        serial_port_name: Name of the serial port for communication. Use `"SIM"` for simulation mode.
        samplerate: Desired sampling rate for acquisition per channel (may not be guaranteed).
        differential: Enable or disable differential mode for the analog input.
        gain: Set the input amplification of the integrated stage.
        offset_enabled: Enable or disable offset removal before amplification (only for single-ended).
        extend_to_int16: Expand the data to 16-bit range and perform crosstalk compensation (experimental).
        realtime_sim: Enable or disable realtime mode during simulation

    Methods:
        start_acquisition(): Starts the data acquisition process.
        stop_acquisition(): Stops the data acquisition process.
        hard_reset(): Performs a hardware reset of the DAQ system using the specified reset pin.
        read_data(): Reads and processes a block of data from the DAQ system.

    Examples:
        >>> from daqopen.duedaq import DueDaq
        >>> my_daq = DueDaq(serial_port_name="SIM")
        >>> my_daq.start_acquisition()
        >>> data = my_daq.read_data()
        >>> print(data)
        >>> my_daq.stop_acquisition()

    Raises:
        DeviceNotFoundException: If the DAQ device is not found.
        DAQErrorException: For errors during data acquisition, such as frame number mismatches.
        AcqNotRunningException: If trying to read data without an active acquisition process.
    """
    MCLK: int = 84_000_000
    CONV_CYCLES_PER_SAMPLE: int = 21
    ADC_PRESCAL_MAX = 255
    MAX_BUFFER_SIZE: int = 20000
    MIN_BUFFER_SIZE: int = 1000
    MAX_BUFFER_DURATION: float = 0.05 # maximum size of buffer in seconds
    NUM_BYTES_PER_SAMPLE: int = 2
    NUM_BYTES_PKG_CNT: int = 4
    START_PATTERN: bytearray = bytearray.fromhex('FFFF')
    FRAME_NUM_DT: np.dtype = np.dtype('uint32')
    FRAME_NUM_DT = FRAME_NUM_DT.newbyteorder('<')
    FRAME_NUM_MAX: int = np.iinfo(FRAME_NUM_DT).max
    ADC_RANGE: list = [0, 4095]
    CHANNEL_MAPPING: dict = {"A0": 7, "A1": 6, "A2": 5, "A3": 4, "A4": 3, "A5": 2,
                             "A6": 1, "A7": 0, "A8": 10, "A9": 11, "A10": 12, "A11": 13}
    CHANNEL_ORDER: list = [ai_pin for ai_pin, ai_mcu in sorted(CHANNEL_MAPPING.items(), key=lambda item: item[1])]

    def __init__(self,
                 channels: list[str] = ["A0"], 
                 reset_pin: int = None, 
                 serial_port_name: int |str = "",
                 samplerate: float = 50000.0, 
                 differential: bool = False, 
                 gain: str | DueDaqGain = DueDaqGain.SGL_1X,
                 offset_enabled: bool = False, 
                 extend_to_int16: bool = False,
                 realtime_sim: bool = True):
        """
    Initialize the DueDaq instance for data acquisition.

    Sets up the necessary configurations for either real or simulated data acquisition. It calculates 
    the ADC prescaler, sets up channels, and initializes the serial communication with either the 
    real Arduino Due hardware or the `DueSerialSim` simulator.

    Parameters:
        channels: List of channels to be acquired.
        reset_pin: GPIO pin number for hardware reset (default: None).
        serial_port_name: Name of the serial port for communication. Use `"SIM"` for simulation mode.
        samplerate: Desired sampling rate for acquisition per channel (cannot be guaranteed).
        differential: Enable or disable differential mode for the analog input.
        gain: Gain of ADC input amplifier (e.g., single-ended 1x, 2x, etc.).
        offset_enabled: Whether to remove offset before amplification (single-ended mode only).
        extend_to_int16: Expand data to 16-bit range and perform crosstalk compensation (experimental).
        realtime_sim: Enable or disable realtime mode during simulation

    Notes:
        - If `serial_port_name` is `"SIM"`, the `DueSerialSim` class will be used for simulated data acquisition.
        - For hardware reset on a Raspberry Pi using `reset_pin`, ensure that the `RPi.GPIO` library is installed.
    """
        if reset_pin is not None:
            try:
                import RPi.GPIO as GPIO
                self._reset_pin = reset_pin
                GPIO.setmode(GPIO.BOARD)
                GPIO.setup(self._reset_pin, GPIO.OUT, initial=GPIO.HIGH)
            except:
                self._reset_pin = None
                logger.warning("GPIO Library not found - not using the reset pin")

        self._adc_channels = channels
        self._wanted_samplerate = samplerate
        self._realtime_sim = realtime_sim
        if isinstance(serial_port_name, str):
            self._serial_port_name = serial_port_name
            self._bind_serial_number = None
        elif isinstance(serial_port_name, int):
            self._serial_port_name = None
            self._bind_serial_number = serial_port_name
        else:
            raise TypeError("No valid type for serial_port_name")
        self._differential = differential
        if not isinstance(gain, DueDaqGain):
            gain = DueDaqGain[gain]
        self._gain = gain
        self._offset_enabled = offset_enabled
        self._extend_to_int16 = extend_to_int16
        # Create map of ai_pin vs. data column index
        data_column = 0
        self.data_columns = {}
        for ai_name in self.CHANNEL_ORDER:
            if ai_name in channels:
                self.data_columns[ai_name] = data_column
                data_column += 1
        self._init_board()
    
    def _init_board(self):
        """Initialize the DAQ board and set up serial communication.

        Depending on the provided `serial_port_name`, this method either initializes a simulation mode 
        using `DueSerialSim` or establishes a connection with the actual Arduino Due hardware via serial.

        It configures the samplerate, buffer size, and prescaler for the ADC, and sends initialization 
        commands to the board, including channel settings, input mode, and gain.

        Raises:
            DeviceNotFoundException: If no Arduino Due device is found.
        """
        # Calculate prescaler
        self._adc_prescal = int(self.MCLK/(self._wanted_samplerate*len(self._adc_channels)*self.CONV_CYCLES_PER_SAMPLE*2)-1)
        self._adc_prescal = min(self._adc_prescal, self.ADC_PRESCAL_MAX)
        self._adc_prescal = max(self._adc_prescal, 1) # Minimum PRESCAL for stable use is 1
        self.samplerate = self.MCLK/((1+self._adc_prescal)*2*self.CONV_CYCLES_PER_SAMPLE*len(self._adc_channels))
        if self.samplerate != self._wanted_samplerate:
            logger.warning(f"Samplerate could not be set to {self._wanted_samplerate:f}. Using {self.samplerate:f} instead")
        # Calculate optimum buffer size
        optimum_buffer_size = int(self.samplerate * self.MAX_BUFFER_DURATION * len(self._adc_channels))
        if (optimum_buffer_size * len(self._adc_channels)) > self.MAX_BUFFER_SIZE:
            optimum_buffer_size = self.MAX_BUFFER_SIZE
        if (optimum_buffer_size * len(self._adc_channels)) < self.MIN_BUFFER_SIZE:
            optimum_buffer_size = self.MIN_BUFFER_SIZE
        self._samples_per_block_channel = optimum_buffer_size // len(self._adc_channels)
        self._dma_buffer_size = self._samples_per_block_channel * len(self._adc_channels)
        # Calculate Blocksize
        self._buffer_blocksize = self.NUM_BYTES_PER_SAMPLE*(len(self._adc_channels)*self._samples_per_block_channel)+self.NUM_BYTES_PKG_CNT+len(self.START_PATTERN)
        # Initialize Interface
        if not self._serial_port_name:
            self._serial_port = self._bind_device() # Update the actual serial port name
        elif self._serial_port_name != "SIM":
            self._serial_port = serial.Serial(self.serial_port_name, timeout=1)
        else:
            self._serial_port = DueSerialSim(self._realtime_sim)
        
        self._read_buffer = bytearray(self._buffer_blocksize)
        self._num_frames_read = 0
        self.daq_data = np.zeros((self._samples_per_block_channel, len(self._adc_channels)), dtype="int16")
        self._acq_state = "stopped"
        # Set Samplerate / Prescaler
        self._serial_port.write((f"SETPRESCAL {self._adc_prescal:d}\n").encode())
        # Enable Channels
        cher = 0
        for ch in self._adc_channels:
            cher |= 1 << self.CHANNEL_MAPPING[ch]
        self._serial_port.write((f"SETCHANNEL {cher:d}\n").encode())
        # Set DMA Buffer Size
        self._serial_port.write((f"SETDMABUFFERSIZE {self._dma_buffer_size:d}\n").encode())
        # Set Input Mode
        if self._differential:
            self._serial_port.write(b"SETMODE 1\n")
        else:
            self._serial_port.write(b"SETMODE 0\n")
        # Set Offset Enabled
        if self._offset_enabled:
            self._serial_port.write(b"SETOFFSET 1\n")
        else:
            self._serial_port.write(b"SETOFFSET 0\n")
        # Set Gain
        self._serial_port.write((f"SETGAIN {self._gain.value:d}\n").encode())
        logger.info("DueDaq Init Done")

    def _bind_device(self):
        """Find the serial port name of the connected Arduino Due device.

        Searches for a connected Arduino Due by checking serial ports for the Vendor ID (VID) 
        and Product ID (PID) specific to the Arduino Due.

        Returns:
            str: The name of the serial port to which the Arduino Due is connected.

        Raises:
            DeviceNotFoundException: If no Arduino Due device is found on the serial ports.
        """
        ports_avail = serial.tools.list_ports.comports()
        for port in ports_avail:
            if port.vid == 0x2341 and port.pid == 0x003e:
                logger.info(f"Device found on Port: {port.device:s}")
                try:
                    serial_port = serial.Serial(port.device, timeout=1)
                    serial_port.write("*IDN?\n".encode("utf-8"))
                    device_id = serial_port.readline().decode().strip()
                    if device_id.startswith("DueDaq"):
                        _, duedaq_version, device_serial_number = device_id.split(",")
                        if (int(device_serial_number) == self._bind_serial_number) or (self._bind_serial_number is None):
                            self._serial_number = int(device_serial_number)
                            logger.info(f"DueDaq with serial {device_serial_number} on {port.device:s}")
                            return serial_port
                    else:
                        logger.warning(f"Not a DueDaq on: {port.device:s}")
                except TimeoutError:
                    logger.warning("No Answer, not sure if device is DueDaq, but still binding")
                    return serial_port
                except IOError:
                    logger.warning(f"Device already in use: {port.device:s}")
        raise DeviceNotFoundException("DueDaq")

    def start_acquisition(self):
        """Start the data acquisition process.

        Sends the "START" command to the DAQ system to begin data acquisition. It waits for the 
        start of a data frame to synchronize the system, then sets the acquisition state to "running".

        Actions:
            - Resets the input buffer.
            - Waits for the first data frame to ensure proper synchronization.
            - Changes the acquisition state to "running".
        """
        self._serial_port.write(b"START\n")
        time.sleep(0.1)
        self._serial_port.reset_input_buffer()
        self._num_frames_read = 0
        logger.info("DueDaq Wait for Frame Start")
        self._wait_for_frame_start()
        self._acq_state = "running"
        logger.info("DueDaq ACQ Started")

    def stop_acquisition(self):
        """Stop the data acquisition process.

        Sends the "STOP" command to the DAQ system to stop data acquisition and clears the 
        input buffer. The state is then set to "stopped".
        """
        self._serial_port.write(b"STOP\n")
        time.sleep(0.1)
        self._serial_port.reset_input_buffer()
        self._acq_state = "stopped"
        logger.info("DueDaq ACQ Stopped")

    def hard_reset(self):
        """Perform a hardware reset of the DAQ system.

        Uses the specified `reset_pin` (if configured) to reset the Arduino Due hardware. This is only 
        applicable when running on a Raspberry Pi with the `RPi.GPIO` library installed.
        
        Actions:
            - Drives the reset pin low for 1 second, then high again to reset the hardware.
            - Reinitializes the DAQ board after the reset.
        """
        if self._reset_pin is None:
            return None
        GPIO.output(self._reset_pin, 0)
        time.sleep(1)
        GPIO.output(self._reset_pin, 1)
        time.sleep(1)
        self._init_board()

    def _wait_for_frame_start(self):
        """Wait for the start of a data frame from the DAQ system.

        Reads incoming data and searches for the `START_PATTERN` to detect the beginning of a valid 
        data frame. This ensures synchronization with the data stream and prevents reading corrupt data.
        """
        prev_byte = bytes.fromhex('00')
        for i in range(10):
            self._serial_port.read(self._buffer_blocksize)
        logger.info("DueDaq Search Start")
        blind_read_bytes = self._buffer_blocksize
        while blind_read_bytes:
            data = self._serial_port.read(1)
            if prev_byte+data == self.START_PATTERN:
                _ = self._serial_port.read(self._buffer_blocksize - len(self.START_PATTERN))
                break
            prev_byte = data
            blind_read_bytes -= 1

    def _read_frame_raw(self):
        """Read a raw data frame from the DAQ system.

        Reads one frame of data from the DAQ system and verifies its integrity by checking the frame 
        number and the start pattern. If the frame number does not increment as expected, an error 
        is raised.

        Raises:
            AcqNotRunningException: If acquisition is not currently running.
            DAQErrorException: If there is a mismatch in the frame number.
        """
        if self._acq_state != "running":
            raise AcqNotRunningException("Can't read frame")
        self._serial_port.readinto(self._read_buffer)
        if self._read_buffer[:len(self.START_PATTERN)] != self.START_PATTERN:
            logger.error('Error Reading Packet')
        # Check if number is increasing
        frame_num = np.frombuffer(self._read_buffer[len(self.START_PATTERN):len(self.START_PATTERN)+self.NUM_BYTES_PKG_CNT], dtype=self.FRAME_NUM_DT)[0]
        if self._num_frames_read == 0:
            self._prev_frame_num = frame_num - 1
            self._num_frames_read += 1
        if frame_num != (self._prev_frame_num + 1) % self.FRAME_NUM_MAX:
            raise DAQErrorException(f"{frame_num:d} != {self._prev_frame_num:d}")
        self._num_frames_read += 1
        self._prev_frame_num = frame_num
        self.daq_data[:] = np.frombuffer(self._read_buffer[len(self.START_PATTERN)+self.NUM_BYTES_PKG_CNT:], dtype='int16').reshape((self._samples_per_block_channel, len(self._adc_channels)))

    def read_data(self) -> np.ndarray:
        """Read and process a block of data from the DAQ system.

        Reads the current frame into the internal buffer, detects and corrects any spikes in the data, 
        and performs necessary adjustments like expanding to 16-bit values and reducing crosstalk.

        Returns:
            The processed data array with dimensions (NUM_SAMPLES, NUM_CH).
        """
        # TODO: Channel Delay Compensation -> not part of this class        
        # Read Frame in Buffer
        self._read_frame_raw()
        if self._differential:
            self.daq_data -= self.ADC_RANGE[1]//2 + 1

        # Expand to 16 Bit
        if self._extend_to_int16:
            # Detect Spikes (random occurance every few hours of acquisition)
            self._correct_adc_spike()
            if self._differential:
                self.daq_data *= 16
                self.daq_data += 8 # Add half of one ADC bit
            else:
                self.daq_data *= 8
            if not self._serial_port_name == "SIM":
                # Reduce Crosstalk (Empirically estimated)
                self.daq_data[:,1] -= (self.daq_data[:,0]/3500).astype(np.int16) # IDX[0] == AD0 IDX[1] == AD2

        return self.daq_data

    def _correct_adc_spike(self):
        """ Correct random spikes generated by ADC

        Detects and corrects spikes in the data that may occur due to ADC anomalies. This is important 
        for maintaining data integrity during long-duration acquisitions.
        """
        for ch_idx in range(len(self._adc_channels)):
            diff = np.diff(self.daq_data[:, ch_idx])
            min_idx = np.argmin(diff)
            max_idx = np.argmax(diff)
            if (abs(min_idx - max_idx) == 1) and (np.sign(self.daq_data[:, ch_idx].max()) != np.sign(self.daq_data[:, ch_idx].min())) and diff.max() > 8:
                spike_data_idx = min(min_idx, max_idx) + 1
                neighbour_diff_idx = spike_data_idx - 2
                if neighbour_diff_idx >= 0:
                    self.daq_data[spike_data_idx, ch_idx] = self.daq_data[spike_data_idx - 1, ch_idx] + diff[neighbour_diff_idx]
                else:
                    self.daq_data[1, ch_idx] = self.daq_data[2, ch_idx] + diff[2]

