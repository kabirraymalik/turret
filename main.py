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
bot.dm.set_position_P(1, 250)
bot.dm.set_position_P(2, 250)
bot.dm.set_position_P(3, 250)
bot.dm.set_position_P(4, 150)
bot.dm.set_position_P(5, 500)
bot.dm.set_position_P(6, 400)
bot.dm.set_position_D(1, 2500)
bot.dm.set_position_D(2, 2500)
bot.dm.set_position_D(3, 2500)
bot.dm.set_position_D(4, 1500)
bot.dm.set_position_D(5, 3200)
bot.dm.set_position_D(6, 3100)

#TODO: velocity control for lift motors? 

bot.assume_position(bot.move_queue[0])

testing = True

while not stopped:
    #runtime limit 15s
    if time.time()-start_time>30:
        stopped = True

    if testing:
        if len(bot.move_queue):
            #if current position = goal position
            if bot.current_position.compare_to(bot.move_queue[0], 0.1):
                #remove first item and assume next position
                bot.move_queue.pop(0)
                bot.assume_position(bot.move_queue[0])
        else:
            bot.go_home()
    
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
