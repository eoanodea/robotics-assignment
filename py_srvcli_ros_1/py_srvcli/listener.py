#!/usr/bin/env python3

import rospy
from std_msgs.msg import String
  
TOPIC_NAME = "topic"
  
def callback(data):
      
    # print the actual message in its raw format
    # rospy.loginfo("Here's what was subscribed: %s", data.data)
      
    # otherwise simply print a convenient message on the terminal
    print('Data from %s received %s', TOPIC_NAME, data.data)
  
  
def main():
      
    # initialize a node by the name 'listener'.
    # you may choose to name it however you like,
    # since you don't have to use it ahead
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber("topic", String, callback)
      
    # spin() simply keeps python from
    # exiting until this node is stopped
    rospy.spin()
  
if __name__ == '__main__':
      
    # you could name this function
    try:
        main()
    except rospy.ROSInterruptException:
        pass