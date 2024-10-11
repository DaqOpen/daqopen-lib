# daqopen/daqzmq.py

"""Module for transferring ADC data via ZeroMQ.

This module provides classes for publishing and subscribing to ADC (Analog-to-Digital Converter) 
data using ZeroMQ sockets. It enables efficient data transfer over a network, allowing for real-time 
communication between data acquisition systems and client applications.

## Usage

The module includes two main classes:
- `DaqPublisher`: Publishes ADC data to a specified TCP address.
- `DaqSubscriber`: Subscribes to ADC data from a specified TCP address.

Examples:
    Publishing ADC data:
    >>> publisher = DaqPublisher(host="127.0.0.1", port=50001)
    >>> publisher.send_data(np.array([1, 2, 3]), packet_num=1, timestamp=1623540000.0)
    >>> publisher.terminate()

    Subscribing to ADC data:
    >>> subscriber = DaqSubscriber(host="127.0.0.1", port=50001)
    >>> metadata, data = subscriber.recv_data()
    >>> subscriber.terminate()

Classes:
    DaqPublisher: Publishes ADC data and metadata over a ZeroMQ socket.
    DaqSubscriber: Subscribes to ADC data and metadata over a ZeroMQ socket.

"""


import numpy as np
import zmq
import logging
import time
from daqopen.daqinfo import DaqInfo

logger = logging.getLogger(__name__)

class DaqPublisher(object):
    """Publishes ADC data and metadata over a ZeroMQ socket.

    `DaqPublisher` is used to send ADC data along with metadata to subscribers over a network 
    using the PUB-SUB pattern of ZeroMQ. It allows for efficient broadcasting of data to multiple 
    clients.

    Attributes:
        zmq_context: The ZeroMQ context for managing socket connections.
        sock: The ZeroMQ PUB socket for data transmission.
        _daq_info: A dictionary containing DAQ configuration information.

    Methods:
        publishObject(data): Publishes a Python object using ZeroMQ.
        send_data(m_data, packet_num, timestamp, sync_status): Sends measurement data and metadata.
        terminate(): Closes the ZeroMQ socket and destroys the context.

    Examples:
        >>> publisher = DaqPublisher(host="127.0.0.1", port=50001)
        >>> publisher.send_data(np.array([1, 2, 3]), packet_num=1, timestamp=1623540000.0)
        >>> publisher.terminate()
    """

    def __init__(self, daq_info: DaqInfo, data_columns: dict, host: str = "127.0.0.1", port: int = 50001):
        """Initialize the DaqPublisher instance.

        Sets up a ZeroMQ PUB socket to publish data to the specified host and port.

        Parameters:
            daq_info: The DaqInfo Object to be published.
            data_columns: The column to channel map.
            host: The IP address to bind the publisher to.
            port: The port number to bind the publisher to.
        """
        self._daq_info = daq_info.to_dict()
        self._data_columns = data_columns
        self.zmq_context = zmq.Context()
        self.sock = self.zmq_context.socket(zmq.PUB)
        self.sock.bind(f"tcp://{host:s}:{port:d}")

    def terminate(self):
        """Terminate the publisher by closing the socket and destroying the context.

        Properly closes the ZeroMQ socket and terminates the context to release resources.
        """
        self.sock.close()
        self.zmq_context.destroy()

    def send_data(self, m_data: np.ndarray, packet_num: int, timestamp: float, sync_status: bool = False) -> int:
        """Send measurement data along with metadata.

        Sends ADC data as a numpy array, accompanied by metadata such as timestamp, packet number, 
        and synchronization status.

        Parameters:
            m_data: The measurement data to be sent.
            packet_num: The packet number for the data.
            timestamp: The timestamp associated with the data.
            sync_status: Indicates if the data is synchronized (default: False).

        Returns:
            Number of bytes sent.
        """
        metadata = dict(
            timestamp = timestamp,
            dtype = str(m_data.dtype),
            shape = m_data.shape,
            daq_info = self._daq_info,
            data_columns = self._data_columns,
            packet_num = packet_num,
            sync_status = sync_status
        )
        self.sock.send_json(metadata, 0|zmq.SNDMORE)
        return self.sock.send(m_data, 0, copy=True, track=False)


class DaqSubscriber(object):
    """Subscribes to ADC data and metadata over a ZeroMQ socket.

    `DaqSubscriber` connects to a ZeroMQ publisher and receives ADC data along with metadata. 
    It allows clients to listen for data broadcasts from a `DaqPublisher`.

    Attributes:
        zmq_context: The ZeroMQ context for managing socket connections.
        sock: The ZeroMQ SUB socket for data reception.

    Methods:
        recv_data(): Receives a numpy array along with its metadata.
        terminate(): Closes the ZeroMQ socket and destroys the context.

    Examples:
        >>> subscriber = DaqSubscriber(host="127.0.0.1", port=50001)
        >>> metadata, data = subscriber.recv_data()
        >>> subscriber.terminate()
    """
    NUM_CONNECT_RETRIES: int = 5

    def __init__(self, host: str = "127.0.0.1", port: int = 50001, init_daqinfo: bool = True, connect_timeout: float = 1.0):
        """Initialize the DaqSubscriber instance.

        Sets up a ZeroMQ SUB socket to receive data from the specified host and port.

        Parameters:
            host: The IP address to connect to.
            port: The port number to connect to.
        """
        self.zmq_context = zmq.Context()
        self.sock = self.zmq_context.socket(zmq.SUB)
        self.sock.setsockopt_string(zmq.SUBSCRIBE, "")
        self.sock.connect(f"tcp://{host:s}:{port:d}")
        self.timestamp: float = -1
        self.daq_info: DaqInfo = DaqInfo.get_default()
        self.data_columns: dict = {}
        self.packet_num: int = -1
        self.sync_status: bool = False
        if init_daqinfo:
            for read_try in range(self.NUM_CONNECT_RETRIES):
                try:
                    self.recv_data(update_daqinfo=True, flags=zmq.NOBLOCK) # Read one packet to update metadata
                    break
                except zmq.ZMQError:
                    logger.warning("Initial receive of data failed")
                time.sleep(connect_timeout/self.NUM_CONNECT_RETRIES)
            else:
                raise ConnectionError


    def recv_data(self, update_daqinfo : bool = False, flags = 0) -> np.ndarray:
        """Receive a numpy array along with its metadata.

        Waits for incoming data and metadata from the publisher, reconstructing the numpy array 
        from the received buffer.

        Returns:
            A tuple containing metadata (dict) and the received numpy array.

        Examples:
            >>> data = subscriber.recv_data()
        """
        metadata = self.sock.recv_json(flags=flags)
        msg = self.sock.recv(flags=flags, copy=True, track=False)
        buf = memoryview(msg)
        daq_data = np.frombuffer(buf, dtype=metadata['dtype'])
        # Update attributes
        self.timestamp = metadata["timestamp"]
        self.packet_num = metadata["packet_num"]
        self.sync_status = metadata["sync_status"]
        if update_daqinfo:
            self.daq_info = DaqInfo.from_dict(metadata["daq_info"])
            self.data_columns = metadata["data_columns"]
        # Return data array
        return daq_data.reshape(metadata['shape'])

    def terminate(self):
        """Terminate the subscriber by closing the socket and destroying the context.

        Properly closes the ZeroMQ socket and terminates the context to release resources.
        """
        self.sock.close()
        self.zmq_context.destroy()