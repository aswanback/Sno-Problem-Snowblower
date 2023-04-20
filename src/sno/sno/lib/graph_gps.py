import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt
from matplotlib.ticker import AutoMinorLocator
import numpy as np

with open('/home/pi/ws/src/sno/sno/lib/gps_data.txt','r') as f:
    y = f.readline()
    y = [float(it) for it in y.strip('][\n').split(', ')]
    x = f.readline()
    x = [float(it) for it in x.strip('][\n').split(', ')]

meter_x = 1/(111111*np.cos(40.4235001/180*np.pi))
meter_y = 1/111111
tol_x = 1*meter_x
tol_y = 1*meter_y
incr = 1

fig, ax = plt.subplots()
ax.scatter(x, y, color='b')
plt.xticks(np.arange(min(x)-tol_x, max(x)+tol_x, incr*meter_x),rotation='vertical')
plt.yticks(np.arange(min(y)-tol_y, max(y)+tol_y, incr*meter_y))
ax.xaxis.set_minor_locator(AutoMinorLocator())

ax.ticklabel_format(useOffset=False)
plt.show()