import os, time
import pygame

if os.name == 'nt':
    import msvcrt

    def getch():
        return mscvrt.getch().decode()
else:
    import sys, tty, termios
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)

    def getch():
        try:
            tty.setraw(sys.stdin.fileno())
            ch = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return ch
    
from dynamixel_sdk import * 

#********* DYNAMIXEL Model definition *********
#***** (Use only one definition at a time) *****
MY_DXL = 'X_SERIES'       # X330 (5.0 V recommended), X430, X540, 2X430
# MY_DXL = 'MX_SERIES'    # MX series with 2.0 firmware update.
# MY_DXL = 'PRO_SERIES'   # H54, H42, M54, M42, L54, L42
# MY_DXL = 'PRO_A_SERIES' # PRO series with (A) firmware update.
# MY_DXL = 'P_SERIES'     # PH54, PH42, PM54
# MY_DXL = 'XL320'        # [WARNING] Operating Voltage : 7.4V


# Control table address
if MY_DXL == 'X_SERIES' or MY_DXL == 'MX_SERIES':
    ADDR_TORQUE_ENABLE          = 64
    ADDR_GOAL_POSITION          = 116
    ADDR_PRESENT_POSITION       = 132
    ADDR_PRESENT_VELOCITY = 128
    ADDR_OPERATING_MODE = 11
    ADDR_GOAL_VELOCITY = 104
    GOAL_VELOCITY1 = 80 #max 265 for 430s
    GOAL_VELOCITY2 = 80
    DXL_MINIMUM_POSITION_VALUE  = 0         # Refer to the Minimum Position Limit of product eManual
    DXL_MAXIMUM_POSITION_VALUE  = 4095      # Refer to the Maximum Position Limit of product eManual
    BAUDRATE                    = 57600
elif MY_DXL == 'PRO_SERIES':
    ADDR_TORQUE_ENABLE          = 562       # Control table address is different in DYNAMIXEL model
    ADDR_GOAL_POSITION          = 596
    ADDR_PRESENT_POSITION       = 611
    DXL_MINIMUM_POSITION_VALUE  = -150000   # Refer to the Minimum Position Limit of product eManual
    DXL_MAXIMUM_POSITION_VALUE  = 150000    # Refer to the Maximum Position Limit of product eManual
    BAUDRATE                    = 57600
elif MY_DXL == 'P_SERIES' or MY_DXL == 'PRO_A_SERIES':
    ADDR_TORQUE_ENABLE          = 512        # Control table address is different in DYNAMIXEL model
    ADDR_GOAL_POSITION          = 564
    ADDR_PRESENT_POSITION       = 580
    DXL_MINIMUM_POSITION_VALUE  = -150000   # Refer to the Minimum Position Limit of product eManual
    DXL_MAXIMUM_POSITION_VALUE  = 150000    # Refer to the Maximum Position Limit of product eManual
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

DXL1_ID = 1
DXL2_ID = 2

DEVICENAME = "/dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_FT94VY5B-if00-port0"

TORQUE_ENABLE = 1
TORQUE_DISABLE = 0
DXL_MOVING_STATUS_THRESHOLD = 20
CHANGE_TO_VELOCITY = 1

index = 0
dxl1_goal_position = [DXL_MINIMUM_POSITION_VALUE,
                      DXL_MAXIMUM_POSITION_VALUE]
dxl1_goal_velocity = GOAL_VELOCITY1
dxl2_goal_velocity = GOAL_VELOCITY2


#motor initialization

# Initialize PortHandler instance
# Set the port path
# Get methods and members of PortHandlerLinux or PortHandlerWindows
portHandler = PortHandler(DEVICENAME)

# Initialize PacketHandler instance
# Set the protocol version
# Get methods and members of Protocol1PacketHandler or Protocol2PacketHandler
packetHandler = PacketHandler(PROTOCOL_VERSION)

# Open port
if portHandler.openPort():
    print("Succeeded to open the port")
else:
    print("Failed to open the port")
    print("Press any key to terminate...")
    getch()
    quit()


# Set port baudrate
if portHandler.setBaudRate(BAUDRATE):
    print("Succeeded to change the baudrate")
else:
    print("Failed to change the baudrate")
    print("Press any key to terminate...")
    getch()
    quit()

dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(
    portHandler, DXL1_ID, ADDR_OPERATING_MODE, CHANGE_TO_VELOCITY)
if dxl_comm_result != COMM_SUCCESS:
    print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
elif dxl_error != 0:
    print("%s" % packetHandler.getRxPacketError(dxl_error))
else:
    print("Dynamixel mode has been successfully changed to velocity")


dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(
    portHandler, DXL1_ID, ADDR_TORQUE_ENABLE, TORQUE_ENABLE)
if dxl_comm_result != COMM_SUCCESS:
    print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
elif dxl_error != 0:
    print("%s" % packetHandler.getRXPacketError(dxl_error))
else:
    print("Dynamixel mode has been successfully connected")

dxl1_goal_velocity = int(input('enter motor velocity: '))

dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(
    portHandler, DXL1_ID, ADDR_GOAL_VELOCITY, dxl1_goal_velocity)
if dxl_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
elif dxl_error != 0:
        print("%s" % packetHandler.getRxPacketError(dxl_error))

time.sleep(5)

dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(
    portHandler, DXL1_ID, ADDR_GOAL_VELOCITY, -dxl1_goal_velocity)
if dxl_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
elif dxl_error != 0:
        print("%s" % packetHandler.getRxPacketError(dxl_error))

global pressed_keys
#loop initialization 
base_refresh_rate = 100 #Hz
pygame.init() #for key listening
display = pygame.display.set_mode((300,300))
events = pygame.event.get()
start_time = time.time()
last_refresh = start_time
stopped = False
while not stopped:
    events = pygame.event.get()
    for event in events:
         if event.type == pygame.KEYDOWN:
            if event.key == pygame.K_w:
                dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(
                        portHandler, DXL1_ID, ADDR_GOAL_VELOCITY, dxl1_goal_velocity)
                if dxl_comm_result != COMM_SUCCESS:
                        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
                elif dxl_error != 0:
                        print("%s" % packetHandler.getRxPacketError(dxl_error))
            if event.key == pygame.K_s:
                dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(
                        portHandler, DXL1_ID, ADDR_GOAL_VELOCITY, -dxl1_goal_velocity)
                if dxl_comm_result != COMM_SUCCESS:
                        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
                elif dxl_error != 0:
                        print("%s" % packetHandler.getRxPacketError(dxl_error))
            elif event.key == pygame.K_ESCAPE:
                stopped = True
            elif event.type == pygame.QUIT:
                 pygame.quit()
                 stopped = True
    #time control 
    while (time.time()) < (last_refresh + 1.0/base_refresh_rate):
        events = pygame.event.get()
        #time.sleep(0.00001)
    curr_time = time.time()
    time_elapsed = curr_time - last_refresh
    last_refresh = curr_time
    print(f'refresh rate: {1.0/time_elapsed} Hz')
pygame.quit()

dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(
    portHandler, DXL1_ID, ADDR_TORQUE_ENABLE, TORQUE_DISABLE)
if dxl_comm_result != COMM_SUCCESS:
    print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
elif dxl_error != 0:
    print("%s" % packetHandler.getRxPacketError(dxl_error))
portHandler.closePort()