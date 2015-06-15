import numpy as np
from numpy import array

def projection(v, z):
    u = array(sorted(v))[::-1]
    l, rho = len(v), 0
    for i in xrange(1, l+1):
        val = u[i-1] - (sum(u[:i]) - z) / i
        if val > 0.0: rho = i
    theta = (sum(u[:rho]) - z) / rho
    w = [max(v[i] - theta, 0) for i in xrange(l)]
    return array(w)

