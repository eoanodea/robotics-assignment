#!/usr/bin/env python3

import rospy
from kobuki_msgs.msg import BumperEvent
from sensor_msgs.msg import Image
from std_msgs.msg import String
  
# TOPIC_NAME = "/mobile_base/events/bumper"
TOPIC_NAME = "/camera/rgb/image_raw"
# TOPIC_NAME = "/topic"
  
# def callback(data):
      
#     # print the actual message in its raw format
#     rospy.loginfo("bumper update %s", data.data)
#     # rospy.loginfo("Here's what was subscribed: bumper: %d state: %d", data.bumper, data.state)
#     #  rospy.loginfo("%s is age: %d" % (data.name, data.age))
      
#     # otherwise simply print a convenient message on the terminal
#     print("data received! %s", data.data)
#     # print('Data from %s received %d', TOPIC_NAME, data.state)

#     # pub = rospy.Publisher('topic2', String, queue_size=10)
#     # rospy.init_node('talker', anonymous=True)
#     # rate = rospy.Rate(10) # 10hz

#     # string_to_publish = "from ros1 " + data.data
#     # pub.publish(string_to_publish)

def callback(data):
  rospy.loginfo("data received! %d", data.width)
  
  
def main():
        # initialize a node by the name 'listener'.
    # you may choose to name it however you like,
    # since you don't have to use it ahead
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber(TOPIC_NAME, Image, callback)
      
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