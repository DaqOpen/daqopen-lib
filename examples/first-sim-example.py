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