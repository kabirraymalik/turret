import cv2, time
import numpy

cap = cv2.VideoCapture(0)

base_refresh_rate = 30 #Hz
start_time = time.time()
last_refresh = start_time
stopped = False

while not stopped:
    #runtime limit 10s
    if time.time()-start_time>10:
        stopped = True

    ret,frame = cap.read() # return a single frame in variable `frame`
    cv2.imshow('Webcam', frame)

    #time control 
    while (time.time()) < (last_refresh + 1.0/base_refresh_rate):
        time.sleep(0.00001)
    curr_time = time.time()
    time_elapsed = curr_time - last_refresh
    last_refresh = curr_time
    #print(f'refresh rate: {1.0/time_elapsed} Hz')
    
cap.release()
cv2.destroyAllWindows()
