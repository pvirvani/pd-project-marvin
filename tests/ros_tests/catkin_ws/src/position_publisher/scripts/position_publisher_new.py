#!/usr/bin/env python

import rospy
from std_msgs.msg import String
import re

def position_publisher():
    pub = rospy.Publisher('lego_data', String, queue_size=10)
    rospy.init_node('position_publisher', anonymous=True)
    rate = rospy.Rate(0.1) #0.1hz : every 10 seconds
    while not rospy.is_shutdown():
        # hello_str = "hello world %s" % rospy.get_time()
        # rospy.loginfo(hello_str)
        # pub.publish(hello_str)
        for n in range(10):
            # pos1 = "p_02_0%s" % str(n)
            # pos2 = "p_02_1%s" % str(n+1)
            # lego = "wc1%s" % str(n)
            pickedpos = ['p_01_01','p_01_02',1,'false']
            placedpos = ['p_05_02','p_06_02',1,'true']
            lego = 'bb5'
            # id = ''.join(filter(str.isdigit, lego))
            id = lego[2:]
            # strings = ''.join(filter(str.isalpha, lego))
            # rospy.logwarn("id is {}, and strings is {}".format(id, strings))
            color= lego[0]
            object=lego[1]
            rospy.logwarn("color = {}, object = {}, and id = {}".format(color, object, id))
            if color == 'b':
                 color = 'blue'
            elif color == 'r':
                 color = 'red'
            elif color == 'g':
                 color = 'green'
            elif color == 'y':
                 color = 'yellow'
            elif color == 'w':
                 color = 'white'
            elif color == 'k':
                 color = 'black'
            elif color == 'p':
                 color = 'purple'
            elif color == 'o':
                 color = 'olive'
            else:
                 color = color
            if object == 'c':
                 object = 'cube'
            elif object == 'b':
                 object = 'brick'
            elif object == 'r':
                 object = 'bar'
            else:
                 object =object                 

            bdata = {'object':object, 'color':color, 'id':id,'pickedfrom':pickedpos, 'placedat':placedpos}
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