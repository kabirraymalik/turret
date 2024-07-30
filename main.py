from dynamixel_utils import *
from kinematics import *

#ssh: ssh kabirraymalik@@192.168.42.59

bot = Eye_Bot()
#loop initialization 
base_refresh_rate = 100 #Hz
start_time = time.time()
last_refresh = start_time
stopped = False

vel1 = bot.dm.get_position_velocity(2)
vel2 = bot.dm.get_position_velocity(3)
#print(f"position velocities of motors 2 and 3: {vel1}, {vel2}")
bot.update_curr_position()
bot.print_robot_info()
bot.get_pid_info(1)
bot.get_pid_info(2)
bot.get_pid_info(3)
bot.dm.set_position_P(1, 300)
bot.dm.set_position_P(2, 300)
bot.dm.set_position_P(3, 300)
bot.dm.set_position_P(4, 300)
bot.dm.set_position_P(5, 500)
bot.dm.set_position_P(6, 400)
bot.dm.set_position_D(1, 3000)
bot.dm.set_position_D(2, 3000)
bot.dm.set_position_D(3, 3000)
bot.dm.set_position_D(4, 3000)
bot.dm.set_position_D(5, 3200)
bot.dm.set_position_D(6, 3100)

bot.go_home()

#TODO: PID Tuning for various modes of movement, also position velocity control 

first_time = True
second_time = True
third_time = True
fourth_time = True
testing = True

while not stopped:
    #runtime limit 10s
    if time.time()-start_time>15:
        stopped = True
        #bot.go_home()

    if testing:
        if first_time and time.time()-start_time>5:
            first_time = False
            bot.assume_position(bot.stored_positions[0])
        if second_time and time.time()-start_time>6:
            second_time = False
            bot.assume_position(bot.stored_positions[1])    
        if third_time and time.time()-start_time>7:
            third_time = False
            bot.assume_position(bot.stored_positions[2])    
        if fourth_time and time.time()-start_time > 8:
            fourth_time = False
            bot.assume_position(bot.stored_positions[3])    
    
    
    
    bot.update_curr_position()
    bot.print_robot_info()
    bot.safety()
    #time control 
    while (time.time()) < (last_refresh + 1.0/base_refresh_rate):
        time.sleep(0.00001)
    curr_time = time.time()
    time_elapsed = curr_time - last_refresh
    last_refresh = curr_time
    #print(f'refresh rate: {1.0/time_elapsed} Hz')
bot.shut_down()
