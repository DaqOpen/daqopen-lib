"""
Module: daqzmq
Description: module for transferring the adc data via zmq

Author: Michael Oberhofer
Created on: 2024-03-15
Last Updated: 2024-03-15

License: MIT

Notes: 

Version: 0.01
Github: https://github.com/DaqOpen/daqopen-lib/daqopen/
"""

import numpy as np
import zmq


class DaqPublisher(object):
    def __init__(self, host: str = "127.0.0.1", port: int = 50001, daq_info: dict = {}):
        self._daq_info = daq_info
        self.zmq_context = zmq.Context()
        self.sock = self.zmq_context.socket(zmq.PUB)
        self.sock.bind(f"tcp://{host:s}:{port:d}")

    def publishObject(self, data):
        self.sock.send_pyobj(data)

    def terminate(self):
        self.sock.close()
        self.zmq_context.destroy()

    def send_data(self, m_data: np.ndarray, packet_num: int, timestamp: float, sync_status: bool = False):
        """ Send Measurement Data together with metadata
        """
        metadata = dict(
            timestamp = timestamp,
            dtype = str(m_data.dtype),
            shape = m_data.shape,
            daq_info = self._daq_info,
            packet_num = packet_num,
            sync_status = sync_status
        )
        self.sock.send_json(metadata, 0|zmq.SNDMORE)
        return self.sock.send(m_data, 0, copy=True, track=False)


class DaqSubscriber(object):
    def __init__(self, host: str = "127.0.0.1", port: int = 50001):
        self.zmq_context = zmq.Context()
        self.sock = self.zmq_context.socket(zmq.SUB)
        self.sock.setsockopt_string(zmq.SUBSCRIBE, "")
        self.sock.connect(f"tcp://{host:s}:{port:d}")

    def recv_data(self):
        """recv a numpy array"""
        metadata = self.sock.recv_json(flags=0)
        msg = self.sock.recv(flags=0, copy=True, track=False)
        buf = memoryview(msg)
        daq_data = np.frombuffer(buf, dtype=metadata['dtype'])
        return metadata, daq_data.reshape(metadata['shape'])

    def terminate(self):
        self.sock.close()
        self.zmq_context.destroy()