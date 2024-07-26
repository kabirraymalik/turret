from dynamixel_sdk import * 
import sys, tty, termios, time
import numpy as np

class DynaManager():
    def __init__(self, device_name, connected_motors):
        self.fd = sys.stdin.fileno()
        self.old_settings = termios.tcgetattr(self.fd)

        #tracks various motor types in system
        self.motors = connected_motors
        #Dynamixel motor pin info
        xl430_info = {'enable_torque': 64,          'set_position': 116,
                    'set_velocity_P': 78,           'get_position': 132, 
                    'get_velocity': 128,            'get_current': 126,
                    'access_operating_mode': 11,    'set_velocity': 104,
                    'max_velocity': 265,            'max_position': 4095,
                    'baudrate': 57600,              'set_position_velocity': 112,
                    'set_position_P': 84,           'set_position_I': 82,
                    'set_position_D': 80,           'set_velocity_I': 76}
        
        xl330_info = {'enable_torque': 64,          'set_position': 116,
                      'set_current': 102,           'get_position': 132, 
                      'get_velocity': 128,          'get_current': 126,
                      'access_operating_mode': 11,  'set_velocity': 104,
                      'max_velocity': 445,          'max_position': 4095,
                      'baudrate': 57600,            'set_position_velocity': 112,
                      'set_position_P': 84,         'set_position_I': 82,
                      'set_position_D': 80,         'set_velocity_I': 76,
                      'set_velocity_P': 78}
        
        self.motor_info = {'XL430': xl430_info, 'XL330': xl330_info}

        # DYNAMIXEL Protocol Version (1.0 / 2.0)
        # https://emanual.robotis.com/docs/en/dxl/protocol2/
        PROTOCOL_VERSION            = 2.0

        self.portHandler = PortHandler(device_name)
        self.packetHandler = PacketHandler(PROTOCOL_VERSION)

        if self.portHandler.openPort():
            print("Succeeded to open the port")
        else:
            print("Failed to open the port")
            print("Press any key to terminate...")
            self.getch()
            quit()

        # Set port baudrate
        if self.portHandler.setBaudRate(int(self.motor_info[self.motors[0]].get('baudrate'))):
            print("Succeeded to change the baudrate")
        else:
            print("Failed to change the baudrate")
            print("Press any key to terminate...")
            self.getch()
            quit()

    #======================================= UTILS ===========================================#

    def get_motor_info(self, motor_ID, cmd):
        if self.motors[motor_ID-1] in self.motor_info:

            info = self.motor_info[self.motors[motor_ID-1]]
            if cmd in info.keys():
                return info[cmd]
            else:
                print('ERROR: invalid command for selected motor!')
                print(f'motor: {motor_ID} command: {cmd}')
                return -1
        else:
            print(f'ERROR: invalid motor type: {self.motors[motor_ID-1]}')
            return -1

    def getch(self):
        try:
            tty.setraw(sys.stdin.fileno())
            ch = sys.stdin.read(1)
        finally:
            termios.tcsetattr(self.fd, termios.TCSADRAIN, self.old_settings)
        return ch
        
    #==============================INDIVIDUAL MOTOR FUNCTIONS=================================#

    def enable_torque(self, motor_ID):
        dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(
            self.portHandler, motor_ID, self.get_motor_info(motor_ID,'enable_torque'), 1)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % self.packetHandler.getRXPacketError(dxl_error))
        else:
            print(f"Dynamixel {motor_ID} torque on")

    def disable_torque(self, motor_ID):
        dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(
            self.portHandler, motor_ID, self.get_motor_info(motor_ID,'enable_torque'), 0)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % self.packetHandler.getRXPacketError(dxl_error))
        else:
            print(f"Dynamixel {motor_ID} torque off")
    
    def set_voltage_mode(self, motor_ID):
        dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(
            self.portHandler, motor_ID, self.get_motor_info(motor_ID,'access_operating_mode'), 16)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % self.packetHandler.getRxPacketError(dxl_error))
        else:
            print(f"Dynamixel {motor_ID} mode successfully changed to current")

    def set_position_mode(self, motor_ID):
        dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(
            self.portHandler, motor_ID, self.get_motor_info(motor_ID,'access_operating_mode'), 3)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % self.packetHandler.getRxPacketError(dxl_error))
        else:
            print(f"Dynamixel {motor_ID} mode successfully changed to position")

    def set_velocity_mode(self, motor_ID):
        dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(
            self.portHandler, motor_ID, self.get_motor_info(motor_ID,'access_operating_mode'), 1)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % self.packetHandler.getRxPacketError(dxl_error))
        else:
            print(f"Dynamixel {motor_ID} mode successfully changed to velocity")
        
    def set_extended_position_control_mode(self, motor_ID):
        dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(
            self.portHandler, motor_ID, self.get_motor_info('access_operating_mode'), 4)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % self.packetHandler.getRxPacketError(dxl_error))
        else:
            print(f"Dynamixel {motor_ID} mode successfully changed to current-based position")

    def set_velocity(self, motor_ID, velocity_as_decimal):
        goal_vel = 265.0 * velocity_as_decimal
        if goal_vel <= 265:
            dxl_comm_result, dxl_error = self.packetHandler.write4ByteTxRx(
                    self.portHandler, motor_ID, self.get_motor_info(motor_ID,'set_velocity'), int(goal_vel))
            if dxl_comm_result != COMM_SUCCESS:
                    print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
            elif dxl_error != 0:
                    print("%s" % self.packetHandler.getRxPacketError(dxl_error))
    
    def get_operating_mode(self, motor_ID):
        dxl_operating_mode, dxl_comm_result, dxl_error = self.packetHandler.read1ByteTxRx(self.portHandler, motor_ID, self.get_motor_info(motor_ID,'access_operating_mode'))
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % self.packetHandler.getRxPacketError(dxl_error))
        else:
            print(f'motor {motor_ID} operating mode: {dxl_operating_mode}')
            return dxl_operating_mode
        
    def set_position(self, motor_ID, position_in_radians):
        dxl_comm_result, dxl_error = self.packetHandler.write4ByteTxRx(self.portHandler, motor_ID, self.get_motor_info(motor_ID,'set_position'), int((self.DXL_MAXIMUM_POSITION_VALUE * position_in_radians/(2 * np.pi))))
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % self.packetHandler.getRxPacketError(dxl_error))

    def get_position(self, motor_ID):
        dxl_present_position, dxl_comm_result, dxl_error = self.packetHandler.read4ByteTxRx(self.portHandler, motor_ID, self.get_motor_info(motor_ID,'get_position'))
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % self.packetHandler.getRxPacketError(dxl_error))
        else:
            return dxl_present_position
    
    def set_current(self, motor_ID, current_as_decimal):
        dxl_comm_result, dxl_error = self.packetHandler.write2ByteTxRx(self.portHandler, motor_ID, self.get_motor_info(motor_ID,'set_current'), current_as_decimal*1750)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % self.packetHandler.getRxPacketError(dxl_error))

    def get_current(self, motor_ID):
        dxl_present_position, dxl_comm_result, dxl_error = self.packetHandler.read4ByteTxRx(self.portHandler, motor_ID, self.get_motor_info(motor_ID,'get_current'))
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % self.packetHandler.getRxPacketError(dxl_error))
        else:
            return (dxl_present_position/self.DXL_MAXIMUM_POSITION_VALUE) * 2 * np.pi
    
     #================================GROUP MOTOR FUNCTIONS====================================#

    def enable_torque_all(self):
        for motor_index in self.motors:
            self.enable_torque(motor_index+1)
    def disable_torque_all(self):
        for motor_index in self.motors:
            self.disable_torque(motor_index+1)
    def shut_down(self):
        self.disable_torque_all()
        self.portHandler.closePort()  

    def go_home(self):
        self.disable_torque(1)
        self.disable_torque(2)
        self.disable_torque(3)
        self.disable_torque(4)
        self.set_position_mode(1)
        self.set_position_mode(2)
        self.set_position_mode(3)
        self.set_position_mode(4)
        self.enable_torque(1)
        self.enable_torque(2)
        self.set_position(1,np.pi)
        self.set_position(2, 0.1)
    def active(self):
        self.set_position(1,3*np.pi/2)
        self.set_position(2, np.pi/3)
    def test1(self):
        self.disable_torque(1)
        self.disable_torque(4)
        self.disable_torque(5)
        self.set_position_mode(1)
        self.set_position_mode(4)
        self.set_position_mode(5)
        self.enable_torque(1)
        self.enable_torque(4)
        self.enable_torque(5)
        self.set_position(1, 0.01)
        self.set_position(4, 0.01)
        self.set_position(5, 0.01)
    def test2(self):
        self.set_position(1, np.pi)
        self.set_position(4, np.pi)
        self.set_position(5, np.pi)