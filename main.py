from dynamixel_utils import *
from kinematics import *

bot = Eye_Bot()
#loop initialization 
base_refresh_rate = 100 #Hz
start_time = time.time()
last_refresh = start_time
stopped = False

bot.go_home()
bot.disable_torque()
vel1 = bot.dm.get_position_velocity(2)
vel2 = bot.dm.get_position_velocity(3)
print(f"velocities of motors 2 and 3: {vel1}, {vel2}")
first_time = True

while not stopped:
    #runtime limit 10s
    if time.time()-start_time>10:
        stopped = True
        #bot.go_home()
    
    if first_time and time.time()-start_time>5:
        first_time = False
        #bot.go_home()
        #bot.set_lift_height(np.pi/3)
        #bot.dm.set_position(1, 3*np.pi/4)
        #bot.dm.set_position(4, np.pi/4)

    bot.read_motor_positions()
    #time control 
    while (time.time()) < (last_refresh + 1.0/base_refresh_rate):
        time.sleep(0.00001)
    curr_time = time.time()
    time_elapsed = curr_time - last_refresh
    last_refresh = curr_time
    print(f'refresh rate: {1.0/time_elapsed} Hz')
bot.shut_down()
