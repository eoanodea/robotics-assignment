#!/usr/bin/env python

import rospy
from std_msgs.msg import String
  
  
def callback_str(data):
    rospy.loginfo("String from ros1 %s", (data.data))
  
def main():
    rospy.init_node('robot_listener', anonymous=True)
    rospy.Subscriber("/robot/topic", String, callback_str)
    
    
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


# scp ./robot/src/src/* ubuntu@192.168.0.107:/home/ubuntu/catkin_ws/src/pub_sub_testing/src/