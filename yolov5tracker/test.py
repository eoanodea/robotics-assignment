import time
import cv2

from yolov5tracker.yolo import yoloTracker

print("Starting main")
mytracker = yoloTracker.plancommand(1)
while True:
    print("command: ", mytracker)
    time.sleep(0.1)