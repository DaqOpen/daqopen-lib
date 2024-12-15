from daqopen.duedaq import DueDaq
import numpy as np
import time
import matplotlib.pyplot as plt

# Create an instance of DueDaq and search for the device
myDaq = DueDaq(serial_port_name="SIM", channels=["A0", "A1"], samplerate=50000, gain="SGL_1X")
print(myDaq._adc_prescal)
print(myDaq._samples_per_block_channel)
print(myDaq._dma_buffer_size)

# Start the acquisition device
myDaq.start_acquisition()

# Remember start timestamp
start_ts = time.time()
number_of_samples = 0

# Read 100 block of data
for i in range(100):
    data = myDaq.read_data()
    number_of_samples += data.shape[0]

# Remember stop timestamp
stop_ts = time.time()

# Stop the acquisition
myDaq.stop_acquisition()

plt.plot(data)
plt.show()

# Calculate Samplerate
samplerate = number_of_samples/(stop_ts - start_ts)
print(f"Samplerate: {samplerate:0.3f}")