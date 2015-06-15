from math import sqrt

vf, au, al, sjam = 31.1, 3.048, -31.7, 6.096


class Vehicle:
    _position, _speed, _destination = 0, 0, 0
    _position_new, _speed_new = 0, 0

    def __init__(self, position, speed, destination):
        self._position = position
        self._speed = speed
        self._destination = destination
        self._position_new = position
        self._speed_new = speed

    def carFollowing(self, container, delta = 1.0):
        if not self.reach(delta):
            precessor = container.findBefore(self)[0]
            pre_vel = precessor._speed
            pre_pos = precessor._position

            if pre_pos <= self._position: pre_pos += container.circle

            delta_x_U = min(vf * delta, self._speed * delta + au * delta * delta)
            delta_x_C = pre_pos - self._position - sjam
            delta_x_L = max(0, self._speed * delta + al * delta * delta);
            d_n_minus_1 = max(0, -pre_vel * pre_vel / 2.0 / al - pre_vel * delta / 2.0)
            delta_x_S = max(0, al * delta * delta / 2 +
                        delta * sqrt(-2.0 * al * ((pre_pos - self._position) + d_n_minus_1)))
            self._position_new = self._position + max(delta_x_L, min(delta_x_U, min(delta_x_S, delta_x_C)))
            self._speed_new = (self._position_new - self._position) / delta

            if self._position_new >= container.circle: self._position_new -= container.circle

        else:
            delta_x_L = max(0, self._speed * delta + al * delta * delta);
            self._position_new = self._position + delta_x_L
            self._speed_new = (self._position_new - self._position) / delta

            if self._position_new >= container.circle: self._position_new -= container.circle


    def fresh(self):
        self._position = self._position_new
        self._speed = self._speed_new

    def reach(self, delta = 1.0):
        destination = self._destination
        if abs(destination - self._position) < 0.5 * vf * delta:
            return True
        else:
            return False

