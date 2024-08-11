import cv2, time
import numpy


def __gstreamer_pipeline(
    camera_id,
    capture_width=1920,
    capture_height=1080,
    display_width=1920,
    display_height=1080,
    framerate=30,
    flip_method=0,
    ):
    return (
            "nvarguscamerasrc sensor-id=%d ! "
            "video/x-raw(memory:NVMM), "
            "width=(int)%d, height=(int)%d, "
            "format=(string)NV12, framerate=(fraction)%d/1 ! "
            "nvvidconv flip-method=%d ! "
            "video/x-raw, width=(int)%d, height=(int)%d, format=(string)BGRx ! "
            "videoconvert ! "
            "video/x-raw, format=(string)BGR ! appsink max-buffers=1 drop=True"
            % (
                    camera_id,
                    capture_width,
                    capture_height,
                    framerate,
                    flip_method,
                    display_width,
                    display_height,
            )
    )
   
#cap = cv2.VideoCapture(__gstreamer_pipeline(camera_id=0, flip_method=2), cv2.CAP_GSTREAMER)

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
