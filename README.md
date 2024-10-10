# daqopen-lib

This library can be used for various data acquisition tasks to proper handle streaming ADC data for building data acquisition applications.

Initially, it is build around the Arduino Due, which has a high-speed ADC with good accuracy and a data transfer via USB 2.0. Most of the examples and driver uses this model together with the firmware which can be found in the firmware folder.

**Documentation** incl. tutorials can be found here: [docs.daqopen.com](https://docs.daqopen.com)

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
import matplotlib.pyplot as plt

# Create Instance of DueDaq
myDaq = DueDaq(serial_port_name="SIM")

# Start acquisition device
myDaq.start_acquisition()

# Read the buffer 10 times
for i in range(10):
    data = myDaq.read_data()

# Hold acqusition device
myDaq.stop_acquisition()

# Plot Data of last buffer
plt.plot(data)
plt.show()
```

![image-20241010124001678](ressources/sim-first-acq.png)



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
import matplotlib.pyplot as plt

# Create Instance of DueDaq (use empty port name for automatic search)
myDaq = DueDaq()

# Start acquisition device
myDaq.start_acquisition()
for i in range(10):
    data = myDaq.read_data() # read buffer

# Hold acqusition device
myDaq.stop_acquisition()

# Plot Data of last buffer
plt.plot(data)
plt.show()
```

You should see something like this:

![my-first-acq-1](ressources/my-first-acq-1.png)

Congratulations!

For more Examples see [docs.daqopen.com](https://docs.daqopen.com)
