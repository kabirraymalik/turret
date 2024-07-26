import numpy as np
import sypy
import math as m

class Eye_Bot():
    def __init__(self):
        self.L1 = 215 #mm
        self.L1_theta_offset = 18 #degrees
        self.L2 = 180 #mm
        self.L2_theta_offset = 12 #degrees, between L1 and L2 like hands on a clock at zero (around 5-10 deg)
        self.rest_position = Position(np.pi/2, 0, -20, 0, 0)


class Position():
    def __init__(self, theta, x, y):
        self.theta = theta
        self.x = x
        self.y = y
        self.wrist = 'none'
        self.tilt = 'none'
    
    def __init__(self, theta, x, y, wrist):
        self.theta = theta
        self.x = x
        self.y = y
        self.wrist = wrist
        self.tilt = 'none'

    def __init__(self, theta, x, y, wrist, tilt):
        self.theta = theta
        self.x = x
        self.y = y
        self.wrist = wrist
        self.tilt = tilt


