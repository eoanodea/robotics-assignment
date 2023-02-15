#!/usr/bin/env python

import rospy
import redis
import os
import base64
import json
import numpy as np

from std_msgs.msg import String
from kobuki_msgs.msg import BumperEvent
from sensor_msgs.msg import Image

REDIS_HOST = os.environ['REDIS_URL']
REDIS_PORT = os.environ['REDIS_PORT']
PERCEPT_CHANNEL = os.environ['PERCEPT_CHANNEL']
COMMAND_CHANNEL = os.environ['COMMAND_CHANNEL']

REDIS_IMAGE_ID = 1
REDIS_HASHSET_NAME = "Image"
  
def callback_str(data):
    rospy.loginfo("String from ros1 %s", (data.data))
    R.publish(PERCEPT_CHANNEL, data.data)

def callback_image(image_data):
    rospy.loginfo("Image data from ROS1")
    data_bytes = bytes(image_data.data)
    base64Image = base64.b64encode(data_bytes)
        
    hash_data = {
        'width': image_data.width,
        'height': image_data.height,
        'data': base64Image
    }

    json_hash_data = json.dumps(hash_data, indent = 4) 
    print('hset')
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

    rospub = rospy.Publisher('/debug/command', String, queue_size=10)
    rospy.init_node('robot_listener', anonymous=True)
    rospy.Subscriber("/debug/percept", String, callback_str)


    rospy.Subscriber("/camera/depth/image_raw", Image, callback_image)

    pubsub = R.pubsub()
    pubsub.subscribe(COMMAND_CHANNEL)

    for msg in pubsub.listen():
        print("Command recieved", msg)
        if msg is not None and isinstance(msg, dict):
            rospub.publish(msg.get('data'))
    
    
    # spin() simply keeps python from
    # exiting until this node is stopped
    rospy.spin()
    
  
if __name__ == '__main__':
      
    # you could name this function
    try:
        main()
    except rospy.ROSInterruptException:
        pass
