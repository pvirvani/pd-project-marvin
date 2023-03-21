#!/usr/bin/env python3.8
# -*- coding: utf-8 -*-
'''
| author:
| Belal HMEDAN, 
| LIG lab/ Marvin Team, 
| France, 2022.
| Vision Model Validation.
'''
import sys
import rospy
from std_msgs.msg import String, Bool
import json
#==========================
# class visionInterpreter |
#==========================

class VisionInterpreter(object):

    def __init__(self, pattern_id):
        """
        Class Vision Interpreter, to interpret the vision data into useful information.
        ---
        Parameters:
        @param: pattern_id, string, "simple" or "complex".
        """
        self.pattern_id = pattern_id
        self.old_dict = ""
        if self.pattern_id == "complex":
            self.assembly_gt = {
                "p_01_08" : [["g", 0]],
                "p_01_09" : [["g", 0]],
                "p_01_10" : [["g", 0]],
                "p_01_11" : [["g", 0]],
                "p_01_12" : [["g", 0]],
                "p_01_13" : [["g", 0]],
                "p_01_14" : [["g", 0]],
                "p_02_07" : [["g", 0]],
                "p_02_08" : [["g", 0], ["w", 1]], 
                "p_02_09" : [["g", 0], ["r", 1], ["l", 2], ["w", 3]], 
                "p_02_10" : [["g", 0], ["b", 1], ["b", 2], ["y", 3], ["b", 4], ["r", 5]], 
                "p_02_11" : [["g", 0], ["b", 1], ["b", 2], ["y", 3], ["o", 4]],
                "p_02_12" : [["g", 0]],
                "p_02_13" : [["g", 0]],
                "p_02_14" : [["g", 0]],
                "p_02_15" : [["g", 0]],
                "p_03_07" : [["g", 0]],
                "p_03_08" : [["g", 0]],
                "p_03_09" : [["g", 0]],
                "p_03_10" : [["g", 0]],
                "p_03_11" : [["g", 0], ["y", 1]],
                "p_03_12" : [["g", 0]],
                "p_03_13" : [["g", 0]],
                "p_03_14" : [["g", 0]],
                "p_03_15" : [["g", 0]],
                "p_04_07" : [["g", 0]],
                "p_04_08" : [["g", 0]],
                "p_04_09" : [["g", 0]],
                "p_04_10" : [["g", 0]],
                "p_04_11" : [["g", 0], ["y", 1]], 
                "p_04_12" : [["g", 0]],
                "p_04_13" : [["g", 0]],
                "p_04_14" : [["g", 0]],
                "p_04_15" : [["g", 0]],
                "p_05_07" : [["g", 0]],
                "p_05_08" : [["g", 0]],
                "p_05_09" : [["g", 0]],
                "p_05_10" : [["g", 0]],
                "p_05_11" : [["g", 0], ["w", 1]], 
                "p_05_12" : [["g", 0], ["r", 1], ["o", 2], ["l", 3]], 
                "p_05_13" : [["g", 0], ["r", 1], ["y", 2], ["b", 3], ["r", 4]], 
                "p_05_14" : [["g", 0], ["y", 1], ["y", 2]],
                "p_05_15" : [["g", 0]],
                "p_06_07" : [["g", 0]],
                "p_06_08" : [["g", 0]],
                "p_06_09" : [["g", 0]],
                "p_06_10" : [["g", 0]],
                "p_06_11" : [["g", 0]],
                "p_06_12" : [["g", 0]],
                "p_06_13" : [["g", 0], ["r", 1], ["w", 2]],
                "p_06_14" : [["g", 0]],
                "p_06_15" : [["g", 0]],
                "p_07_08" : [["g", 0]],
                "p_07_09" : [["g", 0]],
                "p_07_10" : [["g", 0]],
                "p_07_11" : [["g", 0]],
                "p_07_12" : [["g", 0]],
                "p_07_13" : [["g", 0]],
                "p_07_14" : [["g", 0]],
            }
            self.solution = {
                "p_01_08" : ["g", 0],
                "p_01_09" : ["g", 0],
                "p_01_10" : ["g", 0],
                "p_01_11" : ["g", 0],
                "p_01_12" : ["g", 0],
                "p_01_13" : ["g", 0],
                "p_01_14" : ["g", 0],
                "p_02_07" : ["g", 0],
                "p_02_08" : ["w", 1], 
                "p_02_09" : ["w", 3], 
                "p_02_10" : ["r", 5], 
                "p_02_11" : ["o", 4],
                "p_02_12" : ["g", 0],
                "p_02_13" : ["g", 0],
                "p_02_14" : ["g", 0],
                "p_02_15" : ["g", 0],
                "p_03_07" : ["g", 0],
                "p_03_08" : ["g", 0],
                "p_03_09" : ["g", 0],
                "p_03_10" : ["g", 0],
                "p_03_11" : ["y", 1],
                "p_03_12" : ["g", 0],
                "p_03_13" : ["g", 0],
                "p_03_14" : ["g", 0],
                "p_03_15" : ["g", 0],
                "p_04_07" : ["g", 0],
                "p_04_08" : ["g", 0],
                "p_04_09" : ["g", 0],
                "p_04_10" : ["g", 0],
                "p_04_11" : ["y", 1],
                "p_04_12" : ["g", 0],
                "p_04_13" : ["g", 0],
                "p_04_14" : ["g", 0],
                "p_04_15" : ["g", 0],
                "p_05_07" : ["g", 0],
                "p_05_08" : ["g", 0],
                "p_05_09" : ["g", 0],
                "p_05_10" : ["g", 0], 
                "p_05_11" : ["w", 1], 
                "p_05_12" : ["l", 3], 
                "p_05_13" : ["r", 4], 
                "p_05_14" : ["y", 2],
                "p_05_15" : ["g", 0],
                "p_06_07" : ["g", 0],
                "p_06_08" : ["g", 0],
                "p_06_09" : ["g", 0],
                "p_06_10" : ["g", 0],
                "p_06_11" : ["g", 0],
                "p_06_12" : ["g", 0], 
                "p_06_13" : ["w", 2],
                "p_06_14" : ["g", 0],
                "p_06_15" : ["g", 0],
                "p_07_08" : ["g", 0],
                "p_07_09" : ["g", 0],
                "p_07_10" : ["g", 0],
                "p_07_11" : ["g", 0],
                "p_07_12" : ["g", 0],
                "p_07_13" : ["g", 0],
                "p_07_14" : ["g", 0],
            }
        else:
            self.assembly_gt = {
                "p_01_08" : [["g", 0]],
                "p_01_09" : [["g", 0]],
                "p_01_10" : [["g", 0]],
                "p_01_11" : [["g", 0]],
                "p_01_12" : [["g", 0]],
                "p_01_13" : [["g", 0]],
                "p_01_14" : [["g", 0]],
                "p_02_07" : [["g", 0]],
                "p_02_08" : [["g", 0], ["w", 1]],
                "p_02_09" : [["g", 0], ["r", 1]], 
                "p_02_10" : [["g", 0], ["b", 1]], 
                "p_02_11" : [["g", 0], ["b", 1]],
                "p_02_12" : [["g", 0]],
                "p_02_13" : [["g", 0]],
                "p_02_14" : [["g", 0]],
                "p_02_15" : [["g", 0]],
                "p_03_07" : [["g", 0]],
                "p_03_08" : [["g", 0]],
                "p_03_09" : [["g", 0]],
                "p_03_10" : [["g", 0]],
                "p_03_11" : [["g", 0], ["y", 1]],
                "p_03_12" : [["g", 0]],
                "p_03_13" : [["g", 0]],
                "p_03_14" : [["g", 0]],
                "p_03_15" : [["g", 0]],
                "p_04_07" : [["g", 0]],
                "p_04_08" : [["g", 0]],
                "p_04_09" : [["g", 0]],
                "p_04_10" : [["g", 0]],
                "p_04_11" : [["g", 0], ["y", 1]],
                "p_04_12" : [["g", 0]],
                "p_04_13" : [["g", 0]],
                "p_04_14" : [["g", 0]],
                "p_04_15" : [["g", 0]],
                "p_05_07" : [["g", 0]],
                "p_05_08" : [["g", 0]],
                "p_05_09" : [["g", 0]],
                "p_05_10" : [["g", 0]],
                "p_05_11" : [["g", 0], ["w", 1]], 
                "p_05_12" : [["g", 0], ["r", 1]], 
                "p_05_13" : [["g", 0], ["r", 1]], 
                "p_05_14" : [["g", 0], ["y", 1]], 
                "p_05_15" : [["g", 0]],
                "p_06_07" : [["g", 0]],
                "p_06_08" : [["g", 0]],
                "p_06_09" : [["g", 0]],
                "p_06_10" : [["g", 0]],
                "p_06_11" : [["g", 0]],
                "p_06_12" : [["g", 0]], 
                "p_06_13" : [["g", 0], ["r", 1]],
                "p_06_14" : [["g", 0]],
                "p_06_15" : [["g", 0]],
                "p_07_08" : [["g", 0]],
                "p_07_09" : [["g", 0]],
                "p_07_10" : [["g", 0]],
                "p_07_11" : [["g", 0]],
                "p_07_12" : [["g", 0]],
                "p_07_13" : [["g", 0]],
                "p_07_14" : [["g", 0]],
            }
            self.solution = {
                "p_01_08" : ["g", 0],
                "p_01_09" : ["g", 0],
                "p_01_10" : ["g", 0],
                "p_01_11" : ["g", 0],
                "p_01_12" : ["g", 0],
                "p_01_13" : ["g", 0],
                "p_01_14" : ["g", 0],
                "p_02_07" : ["g", 0],
                "p_02_08" : ["w", 1], 
                "p_02_09" : ["r", 1], 
                "p_02_10" : ["b", 1], 
                "p_02_11" : ["b", 1],
                "p_02_12" : ["g", 0],
                "p_02_13" : ["g", 0],
                "p_02_14" : ["g", 0],
                "p_02_15" : ["g", 0],
                "p_03_07" : ["g", 0],
                "p_03_08" : ["g", 0],
                "p_03_09" : ["g", 0],
                "p_03_10" : ["g", 0],
                "p_03_11" : ["y", 1],
                "p_03_12" : ["g", 0],
                "p_03_13" : ["g", 0],
                "p_03_14" : ["g", 0],
                "p_03_15" : ["g", 0],
                "p_04_07" : ["g", 0],
                "p_04_08" : ["g", 0],
                "p_04_09" : ["g", 0],
                "p_04_10" : ["g", 0],
                "p_04_11" : ["y", 1],
                "p_04_12" : ["g", 0],
                "p_04_13" : ["g", 0],
                "p_04_14" : ["g", 0],
                "p_04_15" : ["g", 0],
                "p_05_07" : ["g", 0],
                "p_05_08" : ["g", 0],
                "p_05_09" : ["g", 0],
                "p_05_10" : ["g", 0],
                "p_05_11" : ["w", 1], 
                "p_05_12" : ["r", 1], 
                "p_05_13" : ["r", 1], 
                "p_05_14" : ["y", 1],
                "p_05_15" : ["g", 0],
                "p_06_07" : ["g", 0],
                "p_06_08" : ["g", 0],
                "p_06_09" : ["g", 0],
                "p_06_10" : ["g", 0],
                "p_06_11" : ["g", 0],
                "p_06_12" : ["g", 0], 
                "p_06_13" : ["r", 1],
                "p_06_14" : ["g", 0],
                "p_06_15" : ["g", 0],
                "p_07_08" : ["g", 0],
                "p_07_09" : ["g", 0],
                "p_07_10" : ["g", 0],
                "p_07_11" : ["g", 0],
                "p_07_12" : ["g", 0],
                "p_07_13" : ["g", 0],
                "p_07_14" : ["g", 0],
            }
        self.errors = []
        self.solved = False
        self.vision_dict = None
        
    def compare_world(self):
        """
        Function compare_world, to check the assembly progress/errors.
        ---
        Paramrters:
        @param: None
        
        ---
        @return: None
        """
        self.solved = True
        self.errors = []
        
        for pos in self.assembly_gt:
            # print(pos, '\t', self.vision_dict[pos], '\t', self.assembly_gt[pos])

            if not (self.vision_dict[pos] in self.assembly_gt[pos]):
                self.errors.append(pos)

            if ((self.vision_dict[pos] != self.solution[pos]) or (len(self.errors)>0)):
                self.solved = False
            
    def model_validator(self, data):
        """
        Function model_validator, to validate the assembly progress from vision information.
        ---
        Paramrters:
        @param: data, String, discrete vision representation.
        ---
        @return: None
        """
        self.vision_dict = json.loads(data.data)
        # print(self.errors)
        if(self.vision_dict != self.old_dict):
            self.old_dict = self.vision_dict
            self.compare_world()
            
            errors_string = json.dumps(self.errors)
            self.errors_pub.publish(errors_string)
            self.solved_pub.publish(self.solved)
            self.rate.sleep()

    def receiver(self):
        """
        Function: receiver, to initialize ROS subscriber node to get the vision information.
        ---
        Paramters:
        @param: None
        ---
        @return None
        """
        rospy.init_node('model_validator', anonymous=True)
        self.errors_pub = rospy.Publisher("assembly_errors", String, queue_size=10)
        self.solved_pub = rospy.Publisher("problem_solved", Bool, queue_size=10)
        self.rate = rospy.Rate(10) # 1hz
        rospy.Subscriber('vision_dict', String, self.model_validator)
        rospy.spin()
##===============================================================================

if __name__ == '__main__':
    vision_gui_iface_ = VisionInterpreter(sys.argv[1])
    vision_gui_iface_.receiver()
