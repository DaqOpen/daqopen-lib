import numpy as np
import matplotlib.pyplot as plt
from daqopen.picodaq import PicoDaq

my_daq = PicoDaq()
my_daq.start_acquisition()

data_list = []
for i in range(10):
    data = my_daq.read_data()
    data_list.append(data)
my_daq.stop_acquisition()

data = np.concatenate(data_list)
plt.plot(data)
plt.show()
