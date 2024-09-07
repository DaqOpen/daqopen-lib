"""
Module: duedaq
Description: module for interacting with arduino due daq system

Author: Michael Oberhofer
Created on: 2023-11-15
Last Updated: 2023-11-15

Hardware: Arduino Due (with SAM3X) with due-daq firmware

License: MIT

Notes: 

Version: 0.01
Github: https://github.com/DaqOpen/daqopen-lib/daqopen/
"""

import serial
import serial.tools.list_ports
import time
import numpy as np

class DeviceNotFoundException(Exception):
    def __init__(self, device_name):
        message = f"Device not found: {device_name}"
        super().__init__(message)

class DAQErrorException(Exception):
    def __init__(self, error_info):
        message = f"DAQ Error: {error_info}"
        super().__init__(message)

class AcqNotRunningException(Exception):
    def __init__(self, error_info):
        message = f"Acquisition not Running Error: {error_info}"
        super().__init__(message)

class DueSerialSim(object):
    NUM_BYTES = 2
    NUM_CH = 6
    NUM_SAMPLES = 2048
    BLOCKSIZE = NUM_BYTES*(NUM_CH*NUM_SAMPLES)+2+4
    START_PATTERN = bytearray.fromhex('FFFF')
    FRAME_NUM_DT = np.dtype('uint32')
    FRAME_NUM_DT = FRAME_NUM_DT.newbyteorder('<')
    def __init__(self, packet_generation_delay: float = 0):
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


class DueDaq(object):
    
    NUM_BYTES = 2
    NUM_CH = 6
    NUM_SAMPLES = 2048
    BLOCKSIZE = NUM_BYTES*(NUM_CH*NUM_SAMPLES)+2+4
    START_PATTERN = bytearray.fromhex('FFFF')
    FRAME_NUM_DT = np.dtype('uint32')
    FRAME_NUM_DT = FRAME_NUM_DT.newbyteorder('<')
    FRAME_NUM_MAX = np.iinfo(FRAME_NUM_DT).max
    ADC_RANGE = [-2047, 2048]
    CHANNEL_ORDER = ["AD0", "AD2", "AD4", "AD6", "AD10", "AD12"]
    CHANNEL_PIN_MAPPING = {"AD0": "A7-A6", "AD2": "A5-A4", "AD4": "A3-A2", 
                           "AD6": "A1-A0", "AD10": "A8-A9", "AD12": "A10-A11"}

    def __init__(self, reset_pin: int = None, serial_port_name: str = "", sim_packet_generation_delay: float = 0):
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
        print("DueDaq Init Done")

    def _find_serial_port_name(self):
        ports_avail = serial.tools.list_ports.comports()
        for port in ports_avail:
            if port.vid == 0x2341 and port.pid == 0x003e:
                print(f"Device found on Port: {port.device:s}")
                return port.device
        raise DeviceNotFoundException("DueDaq")

    def _get_input_info(self):
        """ Read the information about the inputs stored in the EEPROM
        """
        pass

    def _set_input_info(self, data):
        """ Store the given data to the EEPROM
        """
        pass

    def start_acquisition(self):
        self._serial_port.write(b"START\n")
        time.sleep(0.1)
        self._serial_port.reset_input_buffer()
        self._num_frames_read = 0
        print("DueDaq Wait for Frame Start")
        self._wait_for_frame_start()
        self._acq_state = "running"
        print("DueDaq ACQ Started")

    def stop_acquisition(self):
        self._serial_port.write(b"STOP\n")
        time.sleep(0.1)
        self._serial_port.reset_input_buffer()
        self._acq_state = "stopped"

    def hard_reset(self):
        if self._reset_pin is None:
            return None
        GPIO.output(self._reset_pin, 0)
        time.sleep(1)
        GPIO.output(self._reset_pin, 1)
        time.sleep(1)
        self._init_board()

    def _wait_for_frame_start(self):
        prev_byte = bytes.fromhex('00')
        for i in range(10):
            self._serial_port.read(self.BLOCKSIZE)
        print("DueDaq Search Start")
        blind_read_bytes = self.BLOCKSIZE
        while blind_read_bytes:
            data = self._serial_port.read(1)
            if prev_byte+data == self.START_PATTERN:
                _ = self._serial_port.read(self.BLOCKSIZE - len(self.START_PATTERN))
                break
            prev_byte = data
            blind_read_bytes -= 1

    def _read_frame_raw(self):
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

    def read_data(self):
        # TODO: Channel Delay Compensation -> not part of this class        
        # Read Frame in Buffer
        self._read_frame_raw()
        # Detect Spikes (random occurance every few hours of acquisition)
        self._correct_adc_spike()
        # Expand to 16 Bit
        self.daq_data -= self.ADC_RANGE[1]
        self.daq_data *= 16
        self.daq_data += 8
        if not self._serial_port_name == "SIM":
            # Reduce Crosstalk (Empirically estimated)
            self.daq_data[:,1] -= (self.daq_data[:,0]/3500).astype(np.int16) # IDX[0] == AD0 IDX[1] == AD2
        return self.daq_data

    def _correct_adc_spike(self):
        """ Correct random spikes generated by ADC
        """
        for ch_idx in range(self.NUM_CH):
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

