import numpy as np
import sypy
import math as m

class Eye_Bot():
    def __init__(self):
        self.L1 = 215 #mm
        self.L1_theta_offset = 18 #degrees
        self.L2 = 180 #mm
        self.L2_theta_offset = 12 #degrees, between L1 and L2 like hands on a clock at zero (around 5-10 deg)
        self.rest_position = Cylindrical_Position(np.pi/2, 0, -20, 0, 0)

    def inverse_cylindrical(self, position):
        #TODO:
        position.update_solution()
    
    def forwards_cylindrical(self, theta_1, theta_2, theta_3, theta_4):
        elbow_pos =  (
            m.cos(theta_2 + m.radians(self.L1_theta_offset)) * self.L1,
            m.sin(theta_2 + m.radians(self.L1_theta_offset)) * self.L1)
        normalized_theta_4 = theta_4 - self.L2_theta_offset - theta_2
        endpoint = (
            elbow_pos(1) - m.cos(normalized_theta_4) * self.L2,
            elbow_pos(2)  + m.sin(normalized_theta_4) * self.L2
        )
        pos = Cylindrical_Position(theta_1, endpoint(1), endpoint(2))
        return pos


class Cylindrical_Position():#
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
    
    def update_solution(self, theta_1, theta_2, theta_3, theta_4):
        self.motor1 = theta_1
        self.motor2 = theta_2
        self.motor3 = theta_3
        self.motor4 = theta_4


