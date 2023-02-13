#!/usr/bin/env python

import rospy
import redis
from std_msgs.msg import String
from kobuki_msgs.msg import BumperEvent
  
  
def callback_str(data):
    rospy.loginfo("String from ros1 %s", (data.data))

def callback_bumper(data): 
    rospy.loginfo("Bumper from ros1 %s", (data.bumper))
    R.publish("/mobile_base/events/bumper", data.bumper)
  
def main():
    global R
    R = redis.Redis(decode_responses=True)
    # pubsub = R.pubsub()

    rospy.init_node('robot_listener', anonymous=True)
    rospy.Subscriber("/mobile_base/events/bumper", BumperEvent, callback_bumper)

    
    
    
    # spin() simply keeps python from
    # exiting until this node is stopped
    rospy.spin()
    
    # rate = rospy.Rate(700) # 10hz

    # while not rospy.is_shutdown():
    #     rate.sleep()

  
if __name__ == '__main__':
      
    # you could name this function
    try:
        main()
    except rospy.ROSInterruptException:
        pass


# scp ./robot/src/* ubuntu@192.168.0.107:/home/ubuntu/catkin_ws/src/pub_sub_testing/src/
# cd ~/catkin_ws
# source devel/setup.bash
# chmod +x listener.py
# roslaunch pub_sub_testing pub_sub_testing.launch
