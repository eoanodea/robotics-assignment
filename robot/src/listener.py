#!/usr/bin/env python

import rospy
import redis
from std_msgs.msg import String
from kobuki_msgs.msg import BumperEvent
import os

REDIS_HOST = os.environ['REDIS_URL']
REDIS_PORT = os.environ['REDIS_PORT']
  
def callback_str(data):
    rospy.loginfo("String from ros1 %s", (data.data))
    R.publish("/debug/percept", data.data)

def callback_bumper(data): 
    rospy.loginfo("Bumper from ros1 %s", (data.bumper))
    R.publish("/mobile_base/events/bumper", data.bumper)
  
def main():
    global R
    R = redis.Redis(host=REDIS_HOST, port=REDIS_PORT, decode_responses=True)

    rospub = rospy.Publisher('/debug/command', String, queue_size=10)
    rospy.init_node('robot_listener', anonymous=True)
    rospy.Subscriber("/debug/percept", String, callback_str)
    rospy.Subscriber("/mobile_base/events/bumper", BumperEvent, callback_bumper)

    pubsub = R.pubsub()
    pubsub.subscribe('/debug/command')

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


# echo export REDIS_URL=localhost >> ~/.bashrc
# echo export REDIS_PORT=6379 >> ~/.bashrc
# echo export 
# scp ./robot/src/* ubuntu@192.168.0.107:/home/ubuntu/catkin_ws/src/pub_sub_testing/src/
# cd ~/catkin_ws
# source devel/setup.bash
# chmod +x listener.py
# catkin_make
# roslaunch pub_sub_testing pub_sub_testing.launch
