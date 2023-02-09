#!/usr/bin/env python3

import rospy
from kobuki_msgs.msg import BumperEvent
from sensor_msgs.msg import Image
from std_msgs.msg import String
  
  
def callback_str(data):
    rospy.loginfo("String from ros2 %s", (data.data))
    pub.publish(data.data)

def callback_bumper(data):
    rospy.loginfo("Bumper from robot: bumper: %d, state: %d", data.state, data.bumper)
    pub2.publish("Bumper from robot: bumper: %d, state: %d", data.state, data.bumper)

  
def main():
    global pub, pub2
    pub = rospy.Publisher('/robot/topic', String, queue_size=10)
    pub2 = rospy.Publisher('/ros2/topic', String, queue_size=10)

    rospy.init_node('ros1_talker_listener')
    rospy.Subscriber("/topic", String, callback_str)
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