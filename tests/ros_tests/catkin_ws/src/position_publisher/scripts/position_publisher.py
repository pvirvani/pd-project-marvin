#!/usr/bin/env python

import rospy
from std_msgs.msg import String

def position_publisher():
    pub = rospy.Publisher('lego_data', String, queue_size=10)
    rospy.init_node('position_publisher', anonymous=True)
    rate = rospy.Rate(0.1) #0.1hz : every 10 seconds
    while not rospy.is_shutdown():
        # hello_str = "hello world %s" % rospy.get_time()
        # rospy.loginfo(hello_str)
        # pub.publish(hello_str)
        for n in range(10):
            pos1 = "p_02_0%s" % str(n)
            pos2 = "p_02_0%s" % str(n+1)
            lego = "white_brick %s" % str(n)
            bdata = {'lego':lego, 'pos1':pos1, 'pos2':pos2}
            rospy.loginfo(str(bdata))
            pub.publish(str(bdata))
            rate.sleep()
        # n = 1
        # pos1 = "p_02_0%s" % str(n)
        # pos2 = "p_02_0%s" % str(n+1)
        # lego = "white_brick %s" % str(n)
        # bdata = [lego, pos1, pos2]
        # rospy.loginfo(str(bdata))
        # pub.publish(str(bdata))
        # rate.sleep()

if __name__ == '__main__':
    try:
        position_publisher()
    except rospy.ROSInterruption:
           pass