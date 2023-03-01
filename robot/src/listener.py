#!/usr/bin/env python

import rospy
import redis
import os
import base64
import json
import time
import threading
import numpy as np

from std_msgs.msg import String
from kobuki_msgs.msg import BumperEvent
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
import cv2

REDIS_HOST = os.environ['REDIS_URL']
REDIS_PORT = os.environ['REDIS_PORT']
PERCEPT_CHANNEL = os.environ['PERCEPT_CHANNEL']
COMMAND_CHANNEL = os.environ['COMMAND_CHANNEL']

REDIS_IMAGE_ID = 1
REDIS_HASHSET_NAME = "Image"
  
def callback_str(data):
    rospy.loginfo("String from ros1 %s", (data.data))
    R.publish(PERCEPT_CHANNEL, data.data)

def cleanup_loop():
    while True:
        cleanup()
        time.sleep(5)

# Any images older than 30 seconds 
# in the hash set is removed
def cleanup():
    if 'R' in globals():
        global REDIS_HASHSET_NAME, R
        keys = R.hkeys(REDIS_HASHSET_NAME)
        if not keys:
            return
        for key in keys:
            value = R.hget(REDIS_HASHSET_NAME, key)
            if not value:
                return
            hash_data = eval(value)
            timestamp = hash_data.get('timestamp', 0)
            current_time = time.time()
            
            if current_time - timestamp >= 5:
                print("[CLEANUP] - Removing old image with ID", key)
                R.hdel(REDIS_HASHSET_NAME, key)


def callback_image(image_data):

    global REDIS_IMAGE_ID

    # Only process every 30th image
    if REDIS_IMAGE_ID % 30 != 0:
        REDIS_IMAGE_ID += 1
        return
    
    print("[ROSPY] - Sending image")


    data_bytes = bytes(image_data.data)
    base64Image = base64.b64encode(data_bytes)
    
    hash_data = {
        'width': image_data.width,
        'height': image_data.height,
        'data': base64Image,
        'timestamp': time.time()
    }

    json_hash_data = json.dumps(hash_data, indent = 4) 

    R.hset(REDIS_HASHSET_NAME, REDIS_IMAGE_ID, json_hash_data)
    
    published_data = {
        'ID': REDIS_IMAGE_ID,
        'type': 'image',
        'hashset': REDIS_HASHSET_NAME
    }

    json_published_data = json.dumps(published_data, indent = 4) 

    R.publish(PERCEPT_CHANNEL, json_published_data)
    
    REDIS_IMAGE_ID = REDIS_IMAGE_ID + 1
  
def main():
    global R
    
    R = redis.Redis(host=REDIS_HOST, port=REDIS_PORT, decode_responses=True)

    #rospub = rospy.Publisher('/debug/command', String, queue_size=10)
    rospub = rospy.Publisher('/mobile_base/commands/velocity', Twist, queue_size=10)
    move_cmd = Twist()


    rospy.init_node('robot_listener', anonymous=True)
    rospy.Subscriber("/debug/percept", String, callback_str)

    rospy.Subscriber("/camera/rgb/image_rect_color", Image, callback_image)
    rate = rospy.Rate(1)

    pubsub = R.pubsub()
    pubsub.subscribe(COMMAND_CHANNEL)

    for msg in pubsub.listen():
        print("Command recieved", msg)
        if msg is not None and isinstance(msg, dict):
            #rospub.publish(msg.get('data'))
            #+ = left, - = right
            reccom = msg.get('data')
            if reccom == 'center':
                move_cmd = Twist()
            elif reccom == 'hard right':
                move_cmd.angular.z = -0.5
            elif reccom == 'hard left':
                move_cmd.angular.z = 0.5
            elif reccom == 'right':
                move_cmd.angular.z = -0.3
            elif reccom == 'left':
                move_cmd.angular.z = 0.3
            else:
                move_cmd = Twist()
            rospub.publish(move_cmd)

    # spin() simply keeps python from
    # exiting until this node is stopped
    rate.sleep()


if __name__ == '__main__':

    # cleanup thread
    cleanup_thread = threading.Thread(target=cleanup_loop)
    cleanup_thread.daemon = True
    cleanup_thread.start()
    
    # main thread
    main()
