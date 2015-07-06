from numpy import array, ones, zeros, mean, exp, std, dot, linspace
from numpy.random import rand, randn
from Vehicle import *
from projection import projection as proj


# the container class, functioning as a circle road

class Container:

    lastControl, lastDescent, lastChange = 0.0, 0.0, 0.0
    road, control, exitSeries = [], [], []
    accumulation, speed, inFlow = [], [], []
    actualInflow, controlPoint = [], []
    accRange = [310,330]#[200,220]#[110,130] #[60.0, 80.0]
    controlVector = ones(8) / 8.0
    controlWeight = ones(8) * 1.4
    accumVector = ones(8)
    totalInflow = 3.8
    gamma, lmbda, alpha = 0.5, 0.9, 3.0
    momentum = zeros(8)


    # initialize circle length
    def __init__(self, l=1000.0):
        self.circle = l


    # normalization
    def Normalize(self, v):
        return v / (1e-6 + sum(v))


    # determine if a vehicle is within the mesuring range
    def within(self, position, interval):
        if interval[1] > interval[0]:
            return position > interval[0] and position < interval[1]
        else:
            return position > interval[0] or position < interval[1]


    # compute the state vector
    def statesVector(self):
        onrampPositions = [i.position for i in self.control]
        shiftedPositions = onrampPositions[1:] + [onrampPositions[0]]
        measureInterval = zip(onrampPositions, shiftedPositions)
        for i, interval in enumerate(measureInterval):
            self.accumVector[i] = 1.0 * len([v for v in self.road if self.within(v._position, interval)])

        self.accumVector = self.Normalize(self.accumVector)


    # Q function
    def Q(self, vector):
        return dot(vector, self.controlWeight)


    # learing Q function
    def sarsa(self, time):

        lastvector = self.InflowVector()
        optimalvector = proj(self.controlWeight, 1.0)

        if rand() < 0.8 * (1 - time / 20000):
            self.controlVector = proj(self.controlWeight + 0.06 * randn(8), 1.0)
        else:
            self.controlVector = optimalvector

        deltaK = self.Q(lastvector) - (self.gamma * self.Q(optimalvector) + self.AverageExitFlow())
        self.momentum = self.lmbda * self.gamma * self.momentum + lastvector
        self.controlWeight = self.controlWeight - self.alpha * self.momentum * deltaK

        return 0


    # stochastic gradient descent, update vector w
    def Approximation(self):
        inflowvector = self.InflowVector()
        error = dot(inflowVector, self.controlWeight) - self.AverageExitFlow()
        self.controlWeight = self.controlWeight - 3.0 * inflowVector * error


    # optimize x based on w, add a e-greedy direction, then do projection
    def DualAscent(self, time):
        optimalDirection = proj(self.controlWeight, 1.0)
        if rand() < 0.8 * (1 - time / 20000):
            self.controlVector = proj(self.controlWeight + 0.05 * randn(8), 1.0)
        else:
            self.controlVector = optimalDirection


    # return the ordered in flow of the past period
    def AverageOrderFlow(self):
        return mean(self.inFlow[-250:])


    # return the actual inflow distribution of the past epoch
    def InflowVector(self):
        res = zeros(8)
        for i in xrange(8):
            res[i] = sum(self.control[i].actualInflow[-250:])
        return res / sum(res)


    # return the total exit flow of the past period
    def AverageExitFlow(self):
        return mean(self.exitSeries[-250:])


    # return the average actual inflow of the past period
    def AverageActualInflow(self):
        return mean(self.actualInflow[-250:])


    # detecte stable states
    def IsStable(self, time):
        end = int(time / 0.98)
        begin = max(0, end - 150)
        if begin > 0 and time - self.lastDescent > 250:
            slices = self.inFlow[begin:end]
            ratio = std(slices) / mean(slices)
            #if ratio < 1.5e-2:
            if True:
                self.lastDescent = time
                return True
            else: return False
        else: return False


    # provide the macro level information of the whole system
    def Information(self):
        num = mean(self.accumulation[-25:])
        deltaAcc = num - mean(self.accumulation[-40:-15])
        overflow = pow(max(0,(num - self.accRange[1])) / self.accRange[1], 0.5)
        underflow = max(0,(self.accRange[0] - num)) / self.accRange[0]
        return (num, deltaAcc, overflow, underflow)


    # should I reduce inflow now?
    def ShouldDecrease(self, num, deltaAcc):
        return num > self.accRange[1] and deltaAcc > 0 #TODO #deltaAcc > 2


    # should I increase inflow now?
    def ShouldIncrease(self, num, deltaAcc):
        return num <= self.accRange[0] or deltaAcc < -1


    # is control already activated
    def IsControlStart(self):
        return self.lastControl > 1.0


    # decreasement of total inflow
    def FlowDecrease(self, overflow, deltaAcc, num):
        return 2.0/(1.0+exp(deltaAcc * 0.025 + overflow * 0.045))


    # increasement of  total inflow
    def FlowIncrease(self, underflow, deltaAcc, num):
        return 0.005*(1.0+exp(max(0,underflow)*0.025+max(0,-deltaAcc)*0.01))/2.0


    # distribute total flow to separated inflows
    def FlowDistribute(self):
        for i in range(len(self.control)):
            self.control[i].flow = self.totalInflow * self.controlVector[i]


    # prevent gridlock
    def ABControl(self,time):
        if time - self.lastControl > 25:
            num, deltaAcc, overflow, underflow = self.Information()
            if self.ShouldDecrease(num, deltaAcc):
                self.lastControl = time
                self.totalInflow *= self.FlowDecrease(overflow, deltaAcc, num)
            if self.ShouldIncrease(num, deltaAcc) and self.IsControlStart():
                self.lastControl = time
                self.totalInflow += self.FlowIncrease(underflow, deltaAcc, num)
            self.FlowDistribute()
        self.inFlow.append(self.totalInflow)


    # compute average speed of the system
    def averageSpeed(self):
        sumV = sum(i._speed for i in self.road)
        numV = len(self.road) + 1.0
        return sumV / numV


    # add a controller to the container, with given information
    def addController(self, controller):
        self.control.append(controller)


    # add a vehicle to the container, in the order of position
    def addVehicle(self, vehicle):
        l = len(self.road)
        rec = l
        for i in xrange(l):
            if self.road[i]._position < vehicle._position: continue
            else: rec = i; break
        if rec==l: self.road.append(vehicle)
        else: self.road.insert(rec, vehicle)


    # excute car following for every vehicle in this container
    def move(self, delta):
        for i in self.road: i.carFollowing(self, delta)


    # fresh the position and speed of every vehicle in this container
    def fresh(self):
        for i in self.road: i.fresh()


    # find the vehicle right before the given one
    def findBefore(self, vehicle):
        roadTmp = sorted(self.road, key = lambda x: x._position)
        for i in range(len(roadTmp)):
            if roadTmp[i]._position > vehicle._position:
                return roadTmp[i], self.road.index(roadTmp[i])
            else: continue
        if len(roadTmp) > 0: return roadTmp[0], self.road.index(roadTmp[0])
        else: return Vehicle(1e8, 0, 1e8), 0


    # remove those vehicles which have reach their destination
    def kickOut(self, time, delta):
        previous = len(self.road)
        self.road = filter(lambda x: not x.reach(delta) or
                self.findController(x._destination).signal(time), self.road)
        self.exitSeries.append(previous - len(self.road))
        self.accumulation.append(len(self.road))
        self.speed.append(self.averageSpeed())


    # return the corresponding controller at the given position
    def findController(self, destination):
        for i in self.control:
            if(i.position == destination):
                return i


    # bunch all the functions together
    def run(self, time, delta):
        actualInflow = 0.0
        for i in self.control:
            actualInflow += i.onRamp(time)
        self.actualInflow.append(actualInflow)
        self.move(delta)
        self.fresh()
        self.kickOut(time, delta)
        self.statesVector()
        if self.IsStable(time):
            self.sarsa(time)
        self.ABControl(time)
