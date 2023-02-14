#!/usr/bin/env python

import rospy
import redis
import os
import base64
import numpy as np

from std_msgs.msg import String
from kobuki_msgs.msg import BumperEvent
from sensor_msgs.msg import Image

REDIS_HOST = os.environ['REDIS_URL']
REDIS_PORT = os.environ['REDIS_PORT']
PERCEPT_CHANNEL = os.environ['PERCEPT_CHANNEL']
COMMAND_CHANNEL = os.environ['COMMAND_CHANNEL']

  
def callback_str(data):
    rospy.loginfo("String from ros1 %s", (data.data))
    R.publish(PERCEPT_CHANNEL, data.data)

def callback_image(image_data):
    rospy.loginfo("Image data from ROS1")
    # Convert the data array to a numpy array of uint8 values
    data_array = np.array(image_data.data, dtype=np.uint8)

    # Assume the image data is stored in a dictionary variable called 'image_data'
    # Convert the 'data' field to a bytes object
    image_bytes = bytes(data_array)

    # Encode the image bytes as base64
    base64_image = base64.b64encode(image_bytes)

    # Print the base64 string
    print(base64_image.decode('utf-8'))

    R.publish(PERCEPT_CHANNEL, base64_image.decode('utf-8'))
  
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
