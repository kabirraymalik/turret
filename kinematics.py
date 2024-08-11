import numpy as np
import math as m
import dynamixel_utils
import time
import os

class Eye_Bot():
    def __init__(self):
        self.motors = ['XL430', 'XL430', 'XL430', 'XL430', 'XL330', 'XL330']
        if os.path.exists('/dev/serial/by-id'):
            device_name = "/dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_FT5NUSV6-if00-port0"
        else:
            device_name = "/dev/tty.usbserial-FT5NUSV6"
        self.dm = dynamixel_utils.DynaManager(device_name=device_name, connected_motors=self.motors)
        
        self.control_mode = ''
        self.torque_enabled = ''

        self.move_queue = []
        self.generate_stored_positions()
        #logs current position
        self.update_curr_position()
        #position accuracy for comparing positions
        self.position_acc = 0.05 #up to 5% off position still ok 

        self.L1 = 215 #mm
        self.L1_theta_offset = 18 #degrees
        self.L2 = 180 #mm
        self.L2_theta_offset = 12 #degrees, between L1 and L2 like hands on a clock at zero (around 5-10 deg)
        self.lift_sync_offset = (6/4095) * 2 * np.pi #difference in angle of lift motors in radians

        self.prev_out = time.time()
    
    def safety(self):
        if self.current_position.theta3 > 5*np.pi/6:
            print("SAFETY BOUND REACHED: TORQUE OFF")
            self.disable_torque()

    def generate_stored_positions(self):
        positions = []
        pos = Position([2.155815865848822, 0.4152765099338252, 1.320012388119247, 2.214074822529949, 4*np.pi/3])
        positions.append(pos)
        pos = Position([3.0825199712146003, 0.4955967897970711, 0.0444963062046906, 3.7023995473075315, 2.3552355180758644])
        positions.append(pos)
        pos = Position([3.1070696573964987, 1.0203463319351465, 0.34369560654657566, 3.9371559214219336, 2.3552355180758644])
        positions.append(pos)
        pos = Position([3.0825199712146003, 0.4955967897970711, 0.0444963062046906, 3.7023995473075315, 2.3552355180758644])
        positions.append(pos)
        pos = Position([4, 0.4152765099338252, 1.320012388119247, 2.214074822529949, 3 * np.pi /4])
        positions.append(pos)
        self.stored_positions = positions
        self.move_queue = positions

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
        pos = Position(theta_1, endpoint(1), endpoint(2))
        return pos
    
    
    def enable_torque(self):
        for motor in range(1, len(self.motors)+1):
            self.dm.enable_torque(motor)
        self.torque_enabled = True

    def disable_torque(self):
        for motor in range(1, len(self.motors)+1):
            self.dm.disable_torque(motor)
        self.torque_enabled = False

    def set_speed_all(self, mode):
        slow = 100
        medium = 150
        fast = 200

        if mode == 'slow':
            for motor in range(1, len(self.motors)+1):
                self.dm.set_position_velocity(motor, slow)
        if mode == 'medium':
            for motor in range(1, len(self.motors)+1):
                self.dm.set_position_velocity(motor, medium)
        if mode == 'fast':
            for motor in range(1, len(self.motors)+1):
                self.dm.set_position_velocity(motor, fast)
    
    def set_mode_all(self, mode):
        self.disable_torque()
        if mode == 'position':
            for motor in range(1, len(self.motors)+1):
                self.dm.set_position_mode(motor)
        elif mode == 'velocity':
            for motor in range(1, len(self.motors)+1):
                self.dm.set_velocity_mode(motor)
        elif mode == 'voltage':
            for motor in range(1, len(self.motors)+1):
                self.dm.set_voltage_mode(motor)
        elif mode == 'extended position control':
            for motor in range(1, len(self.motors)+1):
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
    
    def get_pid_info(self, motor_ID):
        p = self.dm.get_position_P(1)
        i = self.dm.get_position_I(1)
        d = self.dm.get_position_D(1)
        print(f'motor {motor_ID} info: | P: {p} | I: {i} | D: {d} |')
        return [p,i,d]
    
    def read_motor_positions(self):
        output = "|"
        for motor in range(1,len(self.motors)):
            pos = self.dm.get_position(motor)
            output = output + f" motor {motor}: {pos} |"
        print(output)
        return output
    
    def read_motor_positions_rad(self):
        output = "|"
        for motor in range(1,len(self.motors)):
            pos = (self.dm.get_position(motor)/self.dm.get_motor_info(motor, 'max_position'))*2
            output = output + f" motor {motor}: {pos} pi rad |"
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
        if self.control_mode != 'position':
            self.set_mode_all('position')
        if not self.torque_enabled:
            self.torque_enable()
        self.move_motor(1, goal_pos.theta1)
        self.set_lift_height(goal_pos.lift_theta)
        self.move_motor(4, goal_pos.theta3)
        self.move_motor(5, goal_pos.wrist)
        self.move_motor(6, goal_pos.tilt)

        #TODO: this method in position, 0.05 represents percentage accuracy
        if self.current_position.compare_to(goal_pos, 0.05):
            print('robot already in desired position')
    
    def move_motor(self, motor_ID, pos_in_radians):
        if motor_ID == 4 or motor_ID == 5:
            val = 2*np.pi - pos_in_radians
        else:
            val = pos_in_radians
        self.dm.set_position(motor_ID, val)

    def set_lift_height(self, pos_in_radians):
        self.move_motor(2, pos_in_radians + self.lift_sync_offset)
        self.move_motor(3, pos_in_radians)
    
    def go_home(self):
        if self.control_mode != 'position':
            self.set_mode_all('position')
        self.dm.set_position(1,np.pi)
        self.set_lift_height(5*np.pi/180)
        self.move_motor(4, 5*np.pi/180)
        self.move_motor(5, np.pi)
        self.move_motor(6, 0)

    def test_pos(self):
        if self.control_mode != 'position':
            self.set_mode_all('position')
        self.dm.set_position(1,np.pi/2)
    
    def update_curr_position(self):
        self.current_position = Position(self.dm.log_motor_positions())
    
    def print_robot_info(self):
        angles = self.current_position.motor_positions

        theta1 = angles[0]
        lift_theta = angles[1]
        theta3 = angles[2]
        wrist = angles[3]
        tilt = angles[4]
        t = time.time()
        fps = 1/(t - self.prev_out)
        self.prev_out = t
        #print(f'|fps: {fps}| theta: {theta1}| lift: {lift_theta}| theta3 = {theta3}| wrist: {wrist}| tilt: {tilt}|')
        print([theta1, lift_theta, theta3, wrist, tilt])


class Position():
     #only one being used rn
    def __init__(self, motors):
        if len(motors) == 6:
            self.theta1 = motors[0]
            self.lift_theta = motors[1]
            self.theta3 = motors[3]
            self.wrist = motors[4]
            self.tilt = motors[5]
            self.x = None
            self.y = None
        elif len(motors) == 5:
            self.theta1 = motors[0]
            self.lift_theta = motors[1]
            self.theta3 = motors[2]
            self.wrist = motors[3]
            self.tilt = motors[4]
            self.x = None
            self.y = None
        else:
            print('invalid position class initialization')
            return
        self.motor_positions = [self.theta1, self.lift_theta, self.theta3, self.wrist, self.tilt]
    
    def compare_to(self, other_position, accuracy):
        theta1 = abs(self.motor_positions[0] - other_position.motor_positions[0])/self.motor_positions[0]
        lift_theta = abs(self.motor_positions[1] - other_position.motor_positions[1])/self.motor_positions[1]
        theta3 = abs(self.motor_positions[2] - other_position.motor_positions[2])/self.motor_positions[2]
        wrist = abs(self.motor_positions[3] - other_position.motor_positions[3])/self.motor_positions[3]
        tilt = abs(self.motor_positions[4] - other_position.motor_positions[4])/self.motor_positions[4]
        avg_percentage_deviation = (theta1 + lift_theta + theta3 + wrist + tilt)/5
        if avg_percentage_deviation < accuracy:
            return True
        return False


