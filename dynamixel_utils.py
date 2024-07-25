from dynamixel_sdk import * 
import sys, tty, termios, time
import numpy as np

fd = sys.stdin.fileno()
old_settings = termios.tcgetattr(fd)

def getch():
    try:
        tty.setraw(sys.stdin.fileno())
        ch = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    return ch

class Dynamixel_Helper():
    def __init__(self, device_name):

        MY_DXL = 'X_SERIES'       # X330 (5.0 V recommended), X430, X540, 2X430
        # MY_DXL = 'XL320'        # [WARNING] Operating Voltage : 7.4V


        # Control table address
        if MY_DXL == 'X_SERIES' or MY_DXL == 'MX_SERIES':
            self.ADDR_TORQUE_ENABLE          = 64
            self.ADDR_GOAL_POSITION          = 116
            self.ADDR_GOAL_CURRENT           = 102
            self.ADDR_PRESENT_POSITION       = 132
            self.ADDR_PRESENT_VELOCITY = 128
            self.ADDR_PRESENT_CURRENT = 126
            self.ADDR_OPERATING_MODE = 11
            self.ADDR_GOAL_VELOCITY = 104
            GOAL_VELOCITY1 = 80 #max 265 for 430s
            GOAL_VELOCITY2 = 80
            self.DXL_MINIMUM_POSITION_VALUE  = 0         # Refer to the Minimum Position Limit of product eManual
            self.DXL_MAXIMUM_POSITION_VALUE  = 4095      # Refer to the Maximum Position Limit of product eManual
            BAUDRATE                    = 57600
        elif MY_DXL == 'XL320':
            ADDR_TORQUE_ENABLE          = 24
            ADDR_GOAL_POSITION          = 30
            ADDR_PRESENT_POSITION       = 37
            DXL_MINIMUM_POSITION_VALUE  = 0         # Refer to the CW Angle Limit of product eManual
            DXL_MAXIMUM_POSITION_VALUE  = 1023      # Refer to the CCW Angle Limit of product eManual
            BAUDRATE                    = 1000000   # Default Baudrate of XL-320 is 1Mbps

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
            getch()
            quit()

        # Set port baudrate
        if self.portHandler.setBaudRate(BAUDRATE):
            print("Succeeded to change the baudrate")
        else:
            print("Failed to change the baudrate")
            print("Press any key to terminate...")
            getch()
            quit()

    def enable_torque(self, motor_ID):
        dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(
            self.portHandler, motor_ID, self.ADDR_TORQUE_ENABLE, 1)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % self.packetHandler.getRXPacketError(dxl_error))
        else:
            print(f"Dynamixel {motor_ID} torque on")

    def disable_torque(self, motor_ID):
        dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(
            self.portHandler, motor_ID, self.ADDR_TORQUE_ENABLE, 0)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % self.packetHandler.getRXPacketError(dxl_error))
        else:
            print(f"Dynamixel {motor_ID} torque off")
    
    def set_voltage_mode(self, motor_ID):
        dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(
            self.portHandler, motor_ID, self.ADDR_OPERATING_MODE, 16)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % self.packetHandler.getRxPacketError(dxl_error))
        else:
            print(f"Dynamixel {motor_ID} mode successfully changed to current")

    def set_position_mode(self, motor_ID):
        dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(
            self.portHandler, motor_ID, self.ADDR_OPERATING_MODE, 3)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print('dxl_error: {dxl_error}')
            print("%s" % self.packetHandler.getRxPacketError(dxl_error))
        else:
            print(f"Dynamixel {motor_ID} mode successfully changed to position")

    def set_velocity_mode(self, motor_ID):
        dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(
            self.portHandler, motor_ID, self.ADDR_OPERATING_MODE, 1)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % self.packetHandler.getRxPacketError(dxl_error))
        else:
            print(f"Dynamixel {motor_ID} mode successfully changed to velocity")
        
    def set_extended_position_control_mode(self, motor_ID):
        dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(
            self.portHandler, motor_ID, self.ADDR_OPERATING_MODE, 4)
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
                    self.portHandler, motor_ID, self.ADDR_GOAL_VELOCITY, int(goal_vel))
            if dxl_comm_result != COMM_SUCCESS:
                    print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
            elif dxl_error != 0:
                    print("%s" % self.packetHandler.getRxPacketError(dxl_error))
    def get_operating_mode(self, motor_ID):
        dxl_operating_mode, dxl_comm_result, dxl_error = self.packetHandler.read1ByteTxRx(self.portHandler, motor_ID, 11)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % self.packetHandler.getRxPacketError(dxl_error))
        else:
            print(f'motor {motor_ID} operating mode: {dxl_operating_mode}')
            return dxl_operating_mode
        
    def get_position(self, motor_ID):
        dxl_present_position, dxl_comm_result, dxl_error = self.packetHandler.read4ByteTxRx(self.portHandler, motor_ID, self.ADDR_PRESENT_POSITION)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % self.packetHandler.getRxPacketError(dxl_error))
        else:
            return dxl_present_position
        
    def set_position(self, motor_ID, position_in_radians):
        print(f'tried to move to position {int((self.DXL_MAXIMUM_POSITION_VALUE/(2 * np.pi)) * position_in_radians)}')
        dxl_comm_result, dxl_error = self.packetHandler.write4ByteTxRx(self.portHandler, motor_ID, self.ADDR_GOAL_POSITION, int((self.DXL_MAXIMUM_POSITION_VALUE * position_in_radians/(2 * np.pi))))
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % self.packetHandler.getRxPacketError(dxl_error))
    
    def get_current(self, motor_ID):
        dxl_present_position, dxl_comm_result, dxl_error = self.packetHandler.read4ByteTxRx(self.portHandler, motor_ID, self.ADDR_PRESENT_CURRENT)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % self.packetHandler.getRxPacketError(dxl_error))
        else:
            return (dxl_present_position/self.DXL_MAXIMUM_POSITION_VALUE) * 2 * np.pi
        
    def set_current(self, motor_ID, current_as_decimal):
        if motor_ID < 3:
            dxl_comm_result, dxl_error = self.packetHandler.write2ByteTxRx(self.portHandler, motor_ID, 102, current_as_decimal*1750)
            if dxl_comm_result != COMM_SUCCESS:
                print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
            elif dxl_error != 0:
                print("%s" % self.packetHandler.getRxPacketError(dxl_error))
        else: 
            print('figure out current limits for smaller motors')
    def go_home(self):
        self.disable_torque(1)
        self.disable_torque(2)
        self.set_position_mode(1)
        self.set_position_mode(2)
        self.enable_torque(1)
        self.enable_torque(2)
        self.set_position(1,np.pi)
        self.set_position(2, 0.1)
    def active(self):
        self.set_position(1,3*np.pi/2)
        self.set_position(2, np.pi/3)

helper = Dynamixel_Helper(device_name="/dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_FT94VY5B-if00-port0")
helper.go_home()
#loop initialization 
base_refresh_rate = 100 #Hz
start_time = time.time()
last_refresh = start_time
stopped = False
first_time = True
while not stopped:
    if time.time()-start_time > 5:
        if first_time:
            helper.active()
            #helper.set_velocity(2, 0.4)
            first_time = False
    if time.time()-start_time>10:
        stopped = True

    print(f'current position 1: {helper.get_position(1)}')
    #print(f'current position 2: {helper.get_position(2)}')

    #time control 
    while (time.time()) < (last_refresh + 1.0/base_refresh_rate):
        time.sleep(0.00001)
    curr_time = time.time()
    time_elapsed = curr_time - last_refresh
    last_refresh = curr_time
    print(f'refresh rate: {1.0/time_elapsed} Hz')
helper.go_home()
time.sleep()
helper.disable_torque(1)
helper.disable_torque(2)