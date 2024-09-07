# daqopen-lib

This library can be used for various data acquisition tasks to proper handle streaming ADC data for building data acquisition applications.

Initially, it is build around the Arduino Due, which has a high-speed ADC with good accuracy and a data transfer via USB 2.0. Most of the examples and driver uses this model together with the firmware which can be found in the firmware folder.

Further documentation can be soon found here: docs.daqopen.com

## Features

- **ADC driver:** Driver for communicating with Arduino Due (included firmware) and packing the data to numpy arrays.
- **Circular Channel Buffer:** A class representing a circular buffer for holding needed amount of data for viewing, calculating and storing.
- **DAQ-Info Class:** Can be used to exchange informations regarding the interpretation of the data packages. It holds adjustment values and info about the acquisition rate.
- **ZMQ-Support:** Transfer the acquired data in realtime via zmq to other applications or hosts



## Intended Use

This library should be used if:

- you build long-running acquisition applications (e.g. measurement devices)

## Installation

Installation from pypi:

```bash
pip install daqopen
```

Install latest directly from Github:

```
git clone https://github.com/DaqOpen/daqopen-lib.git
cd daqopen-lib
pip install -e .
```

## Usage

### SIM (no hardware)

```python
from daqopen.duedaq import DueDaq

# Create Instance of DueDaq
myDaq = DueDaq(serial_port_name="SIM", sim_packet_generation_delay=0.1)

# Start acquisition device
myDaq.start_acquisition()
for i in range(10):
    data = myDaq.read_data() # read buffer
    print(data)

# Hold acqusition device
myDaq.stop_acquisition()
```



### Arduino Due

#### Setting up Arduino IDE

- Download Arduino IDE for your plattform and start the app
- Install the Package to support SAM-Controllers:  Arduino SAM Boards (32-bits ARM Cortex-
  M3) by Arduino of version **1.6.12**

#### Compiling and Downloading

- Open the sketch-file from firmware/due-daq/due-daq.ino
- Connect the Arduino Due to the "Programming Port" (the port near to the power socket)
- Compile and upload the firmware
- Disconnect from the "Programming Port"



Now, connect the "Native USB Port" (the port near the reset toggle) and use the following sketch for testing the Arduino acquisition:

```python
from daqopen.duedaq import DueDaq

# Create Instance of DueDaq (use empty port name for automatic search)
myDaq = DueDaq(serial_port_name="", sim_packet_generation_delay=0.1)

# Start acquisition device
myDaq.start_acquisition()
for i in range(10):
    data = myDaq.read_data() # read buffer
    print(data)

# Hold acqusition device
myDaq.stop_acquisition()
```

You should see something like this:

![image-20240907121256271](/home/moberhofer/snap/typora/90/.config/Typora/typora-user-images/image-20240907121256271.png)

Congratulations!

See [daqopen-apps](https://github.com/DaqOpen/daqopen-apps) repository for more examples