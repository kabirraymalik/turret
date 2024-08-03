from picamera2 import Picamera2, Preview
from time import sleep

#see also: https://github.com/Gordon999/Pi_Videoer.git

picam2 = Picamera2()
picam2.start_preview(Preview.QTGL)
picam2.start()
sleep(5)
picam2.close()
transform=Transform(hflip=True, vflip=True)

