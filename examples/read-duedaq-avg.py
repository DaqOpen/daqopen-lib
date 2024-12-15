from daqopen.duedaq import DueDaq
import numpy as np
import matplotlib.pyplot as plt

# Create an instance of DueDaq and search for the device
myDaq = DueDaq(serial_port_name="SIM")

# Start the acquisition device
myDaq.start_acquisition()

# Read one block of data
data = myDaq.read_data()

# Stop the acquisition
myDaq.stop_acquisition()

# Calculate average to remove noise
avg_value = data.mean(axis=0)

# Print average
print(f"Value of AD6 (A1-A0): {avg_value[3]:.3f}")

# Scale data
A1_A0 = data[:,3]*0.000088302 + (-0.449022859)

# Plot
plt.plot(A1_A0)
plt.grid()
plt.show()