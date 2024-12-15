# daqopen/picodaq.py

"""Module for interacting with Raspberry Pi Pico DAQ system.

This module provides classes and exceptions for data acquisition from an Raspberry Pi Pico using the pico-daq firmware. 
It enables both actual data acquisition and simulation for testing purposes.

## Usage

The primary class for interacting with the DAQ system is `PicoDaq`. It handles communication with the Arduino Due device, 
starts and stops data acquisition, and retrieves the data collected. For testing without hardware, `DueSerialSim` 
can simulate data acquisition by generating mock data.

Examples:
    Create a PicoDaq instance with a simulated serial port

    >>> from daqopen.picodaq import PicoDaq
    >>> my_daq = PicoDaq(serial_port_name="SIM")
    >>> my_daq.start_acquisition()
    >>> data = my_daq.read_data()
    >>> print(data)
    [100, 100, 100]
    >>> my_daq.stop_acquisition()

Classes:
    DueSerialSim: Simulation Class for testing purpose.
    PicoDaq: Driver class for actual data acquisition.

Raises:
    DeviceNotFoundException: Exception when the device could not be found by its VID and PID.
    DAQErrorException: Exception when there occurs any other error relating the data acquisition.
    AcqNotRunningException: Exception  when data will be read without acquisition running.

"""

import serial
import serial.tools.list_ports
import time
import numpy as np

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
        NUM_BYTES: Number of bytes per data sample.
        NUM_CH: Number of channels available for data acquisition.
        NUM_SAMPLES: Number of samples per data frame.
        BLOCKSIZE: Size of each data block for a single frame.
        START_PATTERN: Byte pattern marking the start of a data frame.
        FRAME_NUM_DT: Data type for frame number with little-endian byte order.

    Parameters:
        packet_generation_delay: Delay in seconds for simulating packet generation (default: 0).

    Methods:
        write: Simulates writing commands to the DAQ system (e.g., "START", "STOP").
        read: Reads a specified length of data from the simulated DAQ system.
        readinto: Reads data directly into the provided buffer.
        reset_input_buffer: Resets the internal read buffer.
        
    Notes:
        Please do not use this class directly, instead use the `PicoDaq` with `serial_port_name = "SIM"`
    """
    NUM_BYTES: int = 2
    NUM_CH: int = 6
    NUM_SAMPLES: int = 2048
    BLOCKSIZE: int = NUM_BYTES*(NUM_CH*NUM_SAMPLES)+2+4
    START_PATTERN: bytearray = bytearray.fromhex('FFFF')
    FRAME_NUM_DT: np.dtype = np.dtype('uint32')
    FRAME_NUM_DT = FRAME_NUM_DT.newbyteorder('<')
    def __init__(self, packet_generation_delay: float = 0):
        """Initialize the DueSerialSim instance for simulating data acquisition.

        This constructor sets up the simulation environment for the Arduino Due DAQ system. 
        It initializes the internal state, prepares the simulated data signals, and sets up the 
        delay between data packet generations.

        Parameters:
            packet_generation_delay: The delay in seconds for simulating the generation of 
                                     data packets. This can be used to mimic the timing 
                                     characteristics of the actual hardware (default: 0).

        Attributes:
            _packet_generation_delay: The delay between packet generations.
            response_data: Placeholder for response data, initialized as an empty byte string.
            _frame_number: Counter to keep track of the frame number.
            _actual_statr: Current state of the simulator, either "started" or "stopped".
            _read_buffer: Buffer to store generated frames for reading.
        """
        self._packet_generation_delay = packet_generation_delay
        self.response_data = b""
        self._frame_number = 0
        self._actual_state = "stopped"
        self._pre_generate_signal()
        self._read_buffer = b""

    def _pre_generate_signal(self):
        index = np.arange(0, self.NUM_SAMPLES)
        main_signal = np.clip(np.sin(2*np.pi*index/self.NUM_SAMPLES) * 2048 + 2048, 0, 4095)
        main_signal[int(self.NUM_SAMPLES/4)] = 0 # Insert Spike for testing
        self._signal_buffer = np.zeros((self.NUM_SAMPLES, self.NUM_CH), dtype="int16")
        for i in range(self.NUM_CH):
            self._signal_buffer[:,i] = main_signal
        #._signal_buffer = self._signal_buffer.flatten()

    def _generate_frame(self):
        time.sleep(self._packet_generation_delay)
        self._frame_number += 1
        frame = self.START_PATTERN+np.array([self._frame_number], dtype=self.FRAME_NUM_DT).tobytes()
        frame += self._signal_buffer.tobytes()
        self._read_buffer = frame

    def write(self, data: bytes):
        if data == b"START\n":
            self._actual_state = "started"
            self._frame_number = 0
        elif data == b"STOP\n":
            self._actual_state = "stopped"
        elif data == b"RESET\n":
            pass
        else:
            pass

    def read(self, length: int = 0):
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
        if self._actual_state == "started":
            if self._read_buffer:
                print("Warning - Buffer not empty before new fillup:", len(self._read_buffer))
            self._generate_frame()
            buffer[:] = self._read_buffer[:]
            self._read_buffer = b""
    
    def reset_input_buffer(self):
        self._read_buffer = b""


class PicoDaq(object):
    """
    Driver class for data acquisition from the Arduino Due DAQ system.

    The `PicoDaq` class interfaces with the Arduino Due running the duq-daq firmware. 
    It handles starting and stopping data acquisition, reading and processing the 
    data collected, and managing communication over the serial interface. Additionally, 
    it supports simulated data acquisition for testing purposes.

    Attributes:
        NUM_BYTES: Number of bytes per data sample.
        NUM_CH: Number of channels available for data acquisition.
        NUM_SAMPLES: Number of samples per data frame.
        BLOCKSIZE: Size of each data block for a single frame.
        START_PATTERN: Byte pattern marking the start of a data frame.
        FRAME_NUM_DT: Data type for frame number, with little-endian byte order.
        FRAME_NUM_MAX: Maximum value for the frame number.
        ADC_RANGE: Range of the ADC values for normalization.
        CHANNEL_ORDER: List of channel names in acquisition order.
        CHANNEL_PIN_MAPPING: Mapping of channel names to their corresponding physical pins.

    Parameters:
        reset_pin: GPIO pin number for hardware reset (default: None).
        serial_port_name: Name of the serial port for communication. Use `"SIM"` for simulation mode.
        sim_packet_generation_delay: Delay in seconds for packet generation in simulation mode (default: 0.04).

    Methods:
        start_acquisition(): Starts the data acquisition process.
        stop_acquisition(): Stops the data acquisition process.
        hard_reset(): Performs a hardware reset of the DAQ system using the specified reset pin.
        read_data(): Reads and processes a block of data from the DAQ system.
        
    Examples:
        >>> from daqopen.PicoDaq import PicoDaq
        # Initialize with the simulation mode
        >>> my_daq = PicoDaq(serial_port_name="SIM")
        # Start data acquisition
        >>> my_daq.start_acquisition()
        # Read and print data
        >>> data = my_daq.read_data()
        >>> print(data)
        # Stop data acquisition
        >>> my_daq.stop_acquisition()

    Notes:
        - To use with actual hardware, provide the correct serial port name and ensure the Arduino Due is connected.
        - In simulation mode (`serial_port_name="SIM"`), data acquisition is simulated using the `DueSerialSim` class.
        - If using the `reset_pin` feature on a Raspberry Pi, ensure the `RPi.GPIO` library is installed.

    Raises:
        DeviceNotFoundException: if the specified DAQ device is not found.
        DAQErrorException: for general errors during data acquisition, such as frame number mismatches.
        AcqNotRunningException: if attempting to read data without an active acquisition process.
    """
    NUM_BYTES: int = 2
    NUM_CH: int = 1
    NUM_SAMPLES: int = 1000
    BLOCKSIZE: int = NUM_BYTES*(NUM_CH*NUM_SAMPLES)+2+4
    START_PATTERN: bytearray = bytearray.fromhex('FFFF')
    FRAME_NUM_DT: np.dtype = np.dtype('uint32')
    FRAME_NUM_DT = FRAME_NUM_DT.newbyteorder('<')
    FRAME_NUM_MAX: int = np.iinfo(FRAME_NUM_DT).max
    ADC_RANGE: list = [-2047, 2048]
    CHANNEL_ORDER: list = ["AD0"]
    CHANNEL_PIN_MAPPING: dict = {"AD0": "P26"}

    def __init__(self, reset_pin: int = None, serial_port_name: str = "", sim_packet_generation_delay: float = 0.04):
        """
        Initialize the PicoDaq instance.

        Parameters:
            reset_pin: GPIO pin number for hardware reset (default: None).
            serial_port_name: Name of the serial port for communication. Use `"SIM"` for simulation mode.
            sim_packet_generation_delay: Delay in seconds for packet generation in simulation mode (default: 0.04).

        Notes:
            - If `serial_port_name` is `"SIM"`, the `DueSerialSim` class will be used for simulated data acquisition.
            - If using `reset_pin` for hardware reset on a Raspberry Pi, ensure the `RPi.GPIO` library is installed.
        """
        if reset_pin is not None:
            try:
                import RPi.GPIO as GPIO
                self._reset_pin = reset_pin
                GPIO.setmode(GPIO.BOARD)
                GPIO.setup(self._reset_pin, GPIO.OUT, initial=GPIO.HIGH)
            except:
                self._reset_pin = None
                print("GPIO Library not found - not using the reset pin")

        self._sim_packet_generation_delay = sim_packet_generation_delay
        self._serial_port_name = serial_port_name
        self._init_board()
    
    def _init_board(self):
        """Initialize the board by setting up the serial communication.

        Depending on whether the simulation mode is active, this method will either create a 
        `DueSerialSim` instance for simulated data acquisition or configure a real serial 
        port connection for hardware communication.

        Raises:
            DeviceNotFoundException: If the Arduino Due device is not found on any serial port.
        """
        if not self._serial_port_name:
            serial_port_name = self._find_serial_port_name() # Update the actual serial port name
        else:
            serial_port_name = self._serial_port_name
        if self._serial_port_name == "SIM":
            self._serial_port = DueSerialSim(self._sim_packet_generation_delay)
        else:
            self._serial_port = serial.Serial(serial_port_name, timeout=1)
        self._read_buffer = bytearray(self.BLOCKSIZE)
        self._num_frames_read = 0
        self.daq_data = np.zeros((self.NUM_SAMPLES, self.NUM_CH), dtype="int16")
        self._acq_state = "stopped"
        print("PicoDaq Init Done")

    def _find_serial_port_name(self):
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
            if port.vid == 0x2e8a and port.pid == 0x000a:
                print(f"Device found on Port: {port.device:s}")
                return port.device
        raise DeviceNotFoundException("PicoDaq")

    def _get_input_info(self):
        """Read the information about the inputs stored in the EEPROM.

        This is a placeholder method for future implementation to read configuration or 
        calibration data stored in the EEPROM of the Arduino Due.
        """
        pass

    def _set_input_info(self, data):
        """Store the given data to the EEPROM.

        This is a placeholder method for future implementation to store configuration or 
        calibration data to the EEPROM of the Arduino Due.

        Parameters:
            data: The data to be stored in the EEPROM.
        """
        pass

    def start_acquisition(self):
        """Start the data acquisition process.

        Sends the "START" command to the DAQ system to begin data acquisition. It resets the 
        input buffer and waits for the first frame to ensure synchronization before switching 
        to the "running" state.
        """
        self._serial_port.write(b"START\n")
        time.sleep(0.1)
        self._serial_port.reset_input_buffer()
        self._num_frames_read = 0
        print("PicoDaq Wait for Frame Start")
        self._wait_for_frame_start()
        self._acq_state = "running"
        print("PicoDaq ACQ Started")

    def stop_acquisition(self):
        """Stop the data acquisition process.

        Sends the "STOP" command to the DAQ system to stop data acquisition and clears the 
        input buffer. The state is then set to "stopped".
        """
        self._serial_port.write(b"STOP\n")
        time.sleep(0.1)
        self._serial_port.reset_input_buffer()
        self._acq_state = "stopped"

    def hard_reset(self):
        """Perform a hardware reset of the DAQ system using the specified reset pin.

        This method uses the GPIO pin (if specified and correctly configured) to reset the 
        Arduino Due hardware. This is only applicable when running on a Raspberry Pi with 
        the `RPi.GPIO` library installed.
        """
        if self._reset_pin is None:
            return None
        GPIO.output(self._reset_pin, 0)
        time.sleep(1)
        GPIO.output(self._reset_pin, 1)
        time.sleep(1)
        self._init_board()

    def _wait_for_frame_start(self):
        """Wait for the start of a data frame.

        Reads incoming data until the start pattern (`START_PATTERN`) is detected, ensuring 
        synchronization with the data stream. This helps avoid corrupted or incomplete data frames.
        """
        prev_byte = bytes.fromhex('00')
        for i in range(10):
            self._serial_port.read(self.BLOCKSIZE)
        print("PicoDaq Search Start")
        blind_read_bytes = self.BLOCKSIZE
        while blind_read_bytes:
            data = self._serial_port.read(1)
            if prev_byte+data == self.START_PATTERN:
                _ = self._serial_port.read(self.BLOCKSIZE - len(self.START_PATTERN))
                break
            prev_byte = data
            blind_read_bytes -= 1

    def _read_frame_raw(self):
        """Read a raw data frame from the DAQ system.

        Reads a single frame of data into the internal buffer. It verifies the integrity of the 
        frame by checking for the start pattern and ensuring the frame number is sequential.

        Raises:
            AcqNotRunningException: If the acquisition is not currently running.
            DAQErrorException: If there is a mismatch in the expected frame number.
        """
        if self._acq_state != "running":
            raise AcqNotRunningException("Can't read frame")
        self._serial_port.readinto(self._read_buffer)
        if self._read_buffer[:2] != self.START_PATTERN:
            print('Error Reading Packet')
        # Check if number is increasing
        frame_num = np.frombuffer(self._read_buffer[2:6], dtype=self.FRAME_NUM_DT)[0]
        if self._num_frames_read == 0:
            self._prev_frame_num = frame_num - 1
            self._num_frames_read += 1
        if frame_num != (self._prev_frame_num + 1) % self.FRAME_NUM_MAX:
            raise DAQErrorException(f"{frame_num:d} != {self._prev_frame_num:d}")
        self._num_frames_read += 1
        self._prev_frame_num = frame_num
        self.daq_data[:] = np.frombuffer(self._read_buffer[6:], dtype='int16').reshape((self.NUM_SAMPLES, self.NUM_CH))

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
        # Expand to 16 Bit
        self.daq_data -= self.ADC_RANGE[1]
        self.daq_data *= 16
        self.daq_data += 8
        return self.daq_data

