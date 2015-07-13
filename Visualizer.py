from Vehicle import *
from Container import *
from Controller import *

import matplotlib.pyplot as plt
import numpy as np
import matplotlib.animation as animation

duration = 20000
road = Container(20000, duration)
ratio = 4.0

road.addController(Controller(road,  20*20, 0.1 * ratio, 645*20, 15.0, 10.0,  7.0))#
road.addController(Controller(road, 145*20, 0.1 * ratio, 770*20, 15.0, 10.0, 16.0))#
road.addController(Controller(road, 270*20, 0.1 * ratio, 520*20, 15.0, 10.0, 14.0))
road.addController(Controller(road, 395*20, 0.1 * ratio, 645*20, 15.0, 10.0,  9.0))
road.addController(Controller(road, 520*20, 0.1 * ratio, 770*20, 15.0, 10.0, 21.0))
road.addController(Controller(road, 645*20, 0.1 * ratio, 895*20, 15.0, 10.0,  2.0))
road.addController(Controller(road, 770*20, 0.1 * ratio,  20*20, 15.0, 10.0,  4.0))
road.addController(Controller(road, 895*20, 0.1 * ratio, 145*20, 15.0, 10.0,  5.0))

def movingaverage(values,window):
    weigths = np.repeat(1.0, window)/window
    smas = np.convolve(values, weigths, 'valid')
    return smas

def circleMapping(container):
    radius = container.circle / np.pi / 2.0
    p = [i._position / radius for i in container.road]
    x = [np.sin(i) * radius for i in p]
    y = [np.cos(i) * radius for i in p]
    return (x, y)

time = 0

#fig, ax = plt.subplots(figsize=(8,8))
#radius = road.circle/np.pi/2 + 1

"""
ax.plot([np.sin(i) * (radius + 10) for i in np.linspace(0,2*np.pi,100)],
        [np.cos(i) * (radius + 10) for i in np.linspace(0,2*np.pi,100)],'k')

ax.plot([np.sin(i) * (radius - 10) for i in np.linspace(0,2*np.pi,100)],
        [np.cos(i) * (radius - 10) for i in np.linspace(0,2*np.pi,100)],'k')


scat, = ax.plot([0, 0, radius, -radius],
        [radius, -radius, 0, 0], 'o')

def update(data):
    scat.set_xdata(data[0])
    scat.set_ydata(data[1])
    return scat,

def data_gen():
    global time
    while True:
        road.run(time, 0.98)
        #road.log(logFile)
        x, y = circleMapping(road)
        time += 0.98
        yield [x, y]

ani = animation.FuncAnimation(fig, update, data_gen, interval = 50)

"""
for i in xrange(duration):
    road.run(time, 0.98)
    time += 0.98

dat_1 = movingaverage(np.array(road.accumulation), 100)
dat_2 = movingaverage(np.array(road.exitSeries), 100)
dat_3 = movingaverage(np.array(road.inFlow),100)
dat_4 = movingaverage(np.array(road.speed),100)
dat_5 = movingaverage(np.array(road.actualInflow),100)
dat_6 = np.array(road.controlPoint)
dat_7 = np.array(road.statusVector)
dat_8 = np.array(road.meteringVector)
dat_9 = np.array(road.aggregatedExit)
zip_dat = zip(dat_1, dat_2)

#y = dat_2
#t = np.linspace(0, y.shape[0], y.shape[0])

#ax.plot(t, y, 'r')

#ax2 = ax.twinx()
#t = np.linspace(0, dat_1.shape[0], dat_1.shape[0])
#ax2.plot(t, dat_1, 'b')

#plt.grid()
#plt.show()

np.savetxt("acc.txt", np.array(dat_1))
np.savetxt("exit.txt", np.array(dat_2))
np.savetxt("in.txt", np.array(dat_3))
np.savetxt("speed.txt", np.array(dat_4))
np.savetxt("controlPoint.txt", dat_6)
np.savetxt("meteringVector.txt", dat_8)
np.savetxt("statusVector.txt", dat_7)
np.savetxt("aggregatedExit.txt", dat_9)
