import os, time
from DynamixelSDK.python.src import *

def main():
    refresh_rate = 100 #Hz
    start_time = time.time()
    last_refresh = start_time
    stopped = False
    while not stopped:
        #TODO: do something here
        #time control 
        while (time.time()) < (last_refresh + 1.0/refresh_rate):
            time.sleep(0.00001)
        curr_time = time.time()
        time_elapsed = curr_time - last_refresh
        last_refresh = curr_time
        print(f'refresh rate: {1.0/time_elapsed} Hz')


main()