#!/usr/bin/env python3

import rospy
from std_msgs.msg import String
from kobuki_msgs.msg import BumperEvent
# kobuki_msgs/DigitalInputEvent
  
TOPIC_NAME = "/mobile_base/events/bumper"
  
def callback(data):
      
    # print the actual message in its raw format
    rospy.loginfo("Here's what was subscribed: %s", data.data)
      
    # otherwise simply print a convenient message on the terminal
    print('Data from %s received %s', TOPIC_NAME, data.data)

    # pub = rospy.Publisher('topic2', String, queue_size=10)
    # rospy.init_node('talker', anonymous=True)
    # rate = rospy.Rate(10) # 10hz

    # string_to_publish = "from ros1 " + data.data
    # pub.publish(string_to_publish)
  
  
def main():
    


    # initialize a node by the name 'listener'.
    # you may choose to name it however you like,
    # since you don't have to use it ahead
    # rospy.Rate(10) # 10hz
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber(TOPIC_NAME, BumperEvent, callback)
      
    # spin() simply keeps python from
    # exiting until this node is stopped
    rospy.spin()
  
if __name__ == '__main__':
      
    # you could name this function
    try:
        main()
    except rospy.ROSInterruptException:
        pass