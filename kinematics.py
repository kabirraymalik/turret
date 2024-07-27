import numpy as np
import math as m
import dynamixel_utils
import time

class Eye_Bot():
    def __init__(self):
        self.motors = ['XL430', 'XL430', 'XL430', 'XL430', 'XL330', 'XL330']
        device_name = "/dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_FT5NUSV6-if00-port0"
        self.dm = dynamixel_utils.DynaManager(device_name=device_name, connected_motors = self.motors)
        
        self.control_mode = ''
        self.move_queue = []
        self.current_position = None

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
    
    
    def enable_torque(self):
        for motor in range(1, len(self.motors)):
            self.dm.enable_torque(motor)

    def disable_torque(self):
        for motor in range(1, len(self.motors)):
            self.dm.disable_torque(motor)

    def set_speed_all(self, mode):
        slow = 100
        medium = 150
        fast = 200

        if mode == 'slow':
            for motor in range(1, len(self.motors)):
                self.dm.set_position_velocity(motor, slow)
        if mode == 'medium':
            for motor in range(1, len(self.motors)):
                self.dm.set_position_velocity(motor, medium)
        if mode == 'fast':
            for motor in range(1, len(self.motors)):
                self.dm.set_position_velocity(motor, fast)
    
    def set_mode_all(self, mode):
        self.disable_torque()
        if mode == 'position':
            for motor in range(1, len(self.motors)):
                self.dm.set_position_mode(motor)
        elif mode == 'velocity':
            for motor in range(1, len(self.motors)):
                self.dm.set_velocity_mode(motor)
        elif mode == 'voltage':
            for motor in range(1, len(self.motors)):
                self.dm.set_voltage_mode(motor)
        elif mode == 'extended position control':
            for motor in range(1, len(self.motors)):
                self.dm.set_extended_position_control_mode(motor)
        else:
            print('ERROR: invalid mode selected')
        self.control_mode = mode
        self.enable_torque()

    def shut_down(self):
        time.sleep(1)
        self.disable_torque()
        time.sleep(1)
        self.dm.portHandler.closePort()
        print('robot successfully shut down')
    
    def read_motor_positions(self):
        output = "|"
        for motor in range(1,len(self.motors)):
            pos = self.dm.get_position(motor)
            output = output + f" motor {motor}: {pos} |"
        print(output)
        return output
    
    def read_hardware_status(self):
        output = "|"
        for motor in range(1,len(self.motors)):
            status = self.dm.read_hardware_status(motor)
            output = output + f" motor {motor}: {status}"
        print(output)
        return output
    
    def assume_position(self, goal_pos):
        #TODO: this method in position, 0.05 represents percentage accuracy
        if self.current_pos.compare_to(goal_pos, 0.05):
            print('robot already in desired position')
    
    def move_motor(self, motor_ID, pos_in_radians):
        if motor_ID == 4:
            val = 2*np.pi - pos_in_radians
        else:
            val = pos_in_radians
        self.dm.set_position(motor_ID, val)

    def set_lift_height(self, pos_in_radians):
        self.move_motor(2, pos_in_radians)
        self.move_motor(3, pos_in_radians)
    
    def go_home(self):
        if self.control_mode != 'position':
            self.set_mode_all('position')
        self.dm.set_position(1,np.pi)
        self.set_lift_height(0)
        self.move_motor(4, 0.5)
        self.dm.set_position(5, 0.5)

    def test_pos(self):
        if self.control_mode != 'position':
            self.set_mode_all('position')
        self.dm.set_position(1,np.pi)
        self.set_lift_height(np.pi/4)
        self.move_motor(4, np.pi/5)
        self.dm.set_position(5, 0.2)


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


