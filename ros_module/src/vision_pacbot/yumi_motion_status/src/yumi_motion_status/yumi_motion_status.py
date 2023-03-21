#!/usr/bin/env python3.8
# -*- coding: utf-8-*-
"""
| author:
| Belal HMEDAN, 
| LIG lab/ Marvin Team, 
| France, 2022.
| YuMi motion status node publisher.
"""

import rospy
from std_msgs.msg import Bool
import requests
import xml.etree.ElementTree as ET
from requests import auth
import threading
import numpy as np

##================
## RWS Interface |
##================
class RWSwrapper(threading.Thread):

    def __init__(self):
        """
        ABB Robot Web Service Interface @see <a href="https://developercenter.robotstudio.com/api/rwsApi/">here</a>.
        """

        self.REST_URI = "http://192.168.125.1"
        self.username = "Default User"
        self.password = "robotics"
        self.session  = requests.Session()
        self.auth = auth.HTTPDigestAuth(self.username, self.password)
        self.session.auth = self.auth

    def get_jointtarget(self, mechUnit):
        """
        Function: get_jointtarget, to get the robot robtarget.
        ---
        Parameters:
        @param: mechUnit, string, 'ROB_L' or 'ROB_R'.
        ---
        @return: jointtarget, list of joint values, jointtarget(Angles in Degrees).
        """
        URI = self.REST_URI+"/rw/motionsystem/mechunits/"+mechUnit+"/jointtarget"

        resp = self.session.get(URI)
        if(resp.status_code != 200):
            print("Exit with Request Error Code: ", resp.status_code)
            return False
        tree = ET.fromstring(resp.content)

        for child in tree[1].iter('*'):
            if child.attrib == {'class': 'rax_1'}:
                rax_1 = float(child.text)
            elif child.attrib == {'class': 'rax_2'}:
                rax_2 = float(child.text)
            elif child.attrib == {'class': 'rax_3'}:
                rax_3 = float(child.text)
            elif child.attrib == {'class': 'rax_4'}:
                rax_4 = float(child.text)
            elif child.attrib == {'class': 'rax_5'}:
                rax_5 = float(child.text)
            elif child.attrib == {'class': 'rax_6'}:
                rax_6 = float(child.text)
            elif child.attrib == {'class': 'eax_a'}:
                eax_a = float(child.text)

        jointtarget = np.array([rax_1, rax_2, rax_3, rax_4, rax_5, rax_6, eax_a])
        jointtarget = np.round(jointtarget, 1)
        return jointtarget

##=======================
## YumiMotion Interface |
##=======================

class YumiMotion(threading.Thread):

    def __init__(self, delay=0.5, motion_thresh=0.1):
        """
        YuMiMotion class, to get the status of the robot motion.
        """
        # RWS Interface
        self.rws = RWSwrapper()
        # No Motion at Start
        self.motion_left  = False
        self.motion_right = False
        self.motion = False
        # Read Initial Joint Values
        self.init_jnts_left  = self.rws.get_jointtarget(mechUnit='ROB_L')
        self.init_jnts_right = self.rws.get_jointtarget(mechUnit='ROB_R')
        # Delay Between two consecutive Joint Values Read.
        self.delay = delay
        # Motion Threshold (in degrees)
        self.motion_thresh = motion_thresh
        # trigger Timers to Detect the Motion After the Delay
        self.t_left  = threading.Timer(self.delay, self.detect_motion, ['ROB_L', self.init_jnts_left])
        self.t_right = threading.Timer(self.delay, self.detect_motion, ['ROB_R', self.init_jnts_right])

    def detect_motion(self, mechUnit, init_jnts):
        """
        Function: detect_motion, to detect if there is a change in the joint values within a given delay.
        ---
        Parameters:
        @param: mechUnit, string, 'ROB_L' or 'ROB_R'.
        @param: init_jnts, nd-array, joint values.
        ---
        Return: None
        """
        # Read Current Joint Values
        curr_jnts = self.rws.get_jointtarget(mechUnit)
        # Measure the Difference with the Intiial Joint Values
        diff = np.abs(curr_jnts - init_jnts)
        # rospy.logwarn(mechUnit)
        # rospy.logwarn(diff)
        # Apply Thresolding to Avoid Noise
        diff[diff<self.motion_thresh] = 0
        # If There is a different in any Joint Values
        if(np.count_nonzero(diff)>0):
            # Motion Detected
            self.motion = True
            # Left Arm is Moving
            if mechUnit[-1] == 'L':
                self.motion_left = True
                self.init_jnts_left = curr_jnts
            # Right Arm is Moving
            else:
                self.init_jnts_right = True
                self.init_jnts_right = curr_jnts

        else:
            if mechUnit[-1] == 'L':
                self.motion_left = False
                if self.motion_right == False:
                    self.motion =False
            else:
                self.motion_right  = False
                if self.motion_left == False:
                    self.motion =False
        # print("\n======\n", mechUnit, "\n---------\n", diff, "\n0----{}----0\n".format(str(self.motion)))

    def run(self):
        """
        Function: run, to detect if there is a change in the joint values within a given delay.
        ---
        Parameters:
        @param: mechUnit, string, 'ROB_L' or 'ROB_R'.
        @param: init_jnts, nd-array, joint values.
        ---
        Return: None
        """
        if not self.t_left.is_alive():
            self.t_left  = threading.Timer(self.delay, self.detect_motion, ['ROB_L', self.init_jnts_left])
            self.t_left.start()

        if not self.t_right.is_alive():
            self.t_right = threading.Timer(self.delay, self.detect_motion, ['ROB_R', self.init_jnts_right])
            self.t_right.start()

#----------------------------------------------------------------------------------------------------------------
#             
def motion_callback():
    motion_pub = rospy.Publisher("yumi_motion_status", Bool, queue_size=10)
    rospy.init_node("motion_status", anonymous=True)
    rate = rospy.Rate(5)  # 5hz
    try:
        ym = YumiMotion()
        latch = 0
        th_ = 30
        latched_motion = True
        while not rospy.is_shutdown():
            ym.run()
            # rospy.logwarn(ym.motion)
            rospy.logwarn(latch)
            if(ym.motion and latch<th_):
                latch=th_
            elif(not ym.motion and latch>0):
                latch-=1
            if(latch==0):
                latched_motion = False
            else:
                latched_motion = True
            motion_pub.publish(latched_motion)
            rate.sleep()
    except Exception as e:
        rospy.logwarn(e)
        while not rospy.is_shutdown():
            motion_pub.publish(False)
            rate.sleep()
if __name__ == "__main__":
    try:
        motion_callback()
    except rospy.ROSInterruptException:
        pass
