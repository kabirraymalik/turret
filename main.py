from dynamixel_utils import *

motors = ['XL430', 'XL430', 'XL430', 'XL430', 'XL330', 'XL330']
device_name = "/dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_FT5NUSV6-if00-port0"
dm = DynaManager(device_name=device_name, connected_motors = motors)

#loop initialization 
base_refresh_rate = 100 #Hz
start_time = time.time()
last_refresh = start_time
stopped = False
dm.enable_torque_all()
while not stopped:
    #runtime limit 10s
    if time.time()-start_time>10:
        stopped = True


    #time control 
    while (time.time()) < (last_refresh + 1.0/base_refresh_rate):
        time.sleep(0.00001)
    curr_time = time.time()
    time_elapsed = curr_time - last_refresh
    last_refresh = curr_time
    print(f'refresh rate: {1.0/time_elapsed} Hz')
dm.shut_down()

