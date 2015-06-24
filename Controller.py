from Vehicle import *
from Container import *

class Controller:


    def __init__(self, container, position, flow, destination, green, red, initial):
        self.container = container
        self.position = position
        self.flow = flow
        self.destination = destination
        self.lastEnter = 0
        self.green = green
        self.red = red
        self.initial = initial
        self.actualInflow = []


    def onRamp(self, time):
        res = 0.0
        if self.signal(time) and 1.0 / (1e-8 + self.flow) < time - self.lastEnter:
            a = Vehicle(self.position, 0, 0)
            b, bi = self.container.findBefore(a)
            if b._position < a._position:
                bPosition = b._position + self.container.circle
            else: bPosition = b._position
            if sjam < bPosition - a._position:
                self.container.road.insert(bi, Vehicle(self.position,
                    b._speed, self.destination))
                self.lastEnter = time
                res = 1.0
        self.actualInflow.append(res)
        return res


    def signal(self, time):
        local = time + self.initial
        div = int(local / (self.red + self.green))
        res = local - (self.red + self.green) * div
        if res < self.green: return True
        else: return False
