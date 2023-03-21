#!/usr/bin/env python3.8
# -*- coding: utf-8 -*-
'''
| author:
| Belal HMEDAN,
| LIG lab/ Marvin Team,
| France, 2022.
| Vision Model Accumulator.
'''
import rospy
from std_msgs.msg import String
import json
import sys
#==========================
# class ModelAcumulator   |
#==========================

class ModelAcumulator:

    def __init__(self, path):
        """
        Class ModelAcumulator, to accumulate the vision data of all levels.
        """
        self.path = path
        self.model = None
        self.latcher = 0
        self.extracted_model = {}
        self.loadData()

    def loadData(self):
        """
        Function: loadData, to load the model from JSON.
        ---
        Parameters:
        @param: None
        ---
        @return: None
        """
        with open(self.path+'extracted_model.json', 'r') as fp:
            self.extracted_model = json.load(fp)

    def dumpData(self):
        """
        Function: dumpData, to store 3D model into JSON.
        ---
        Parameters:
        @param: None
        ---
        @return: None
        """
        with open(self.path+'extracted_model.json', 'w') as fp:
            json.dump(self.extracted_model, fp)

    def max_non_green(self, dict):
        """

        """
        for i in range(5, 0, -1):
            if(dict[i]!="g"):
                return i
        return 0

    def copy_model(self, pos, level):
        """

        """
        leve = self.max_non_green(self.model[pos])
        if(leve != 0 and self.model[pos][level]!="g"):
            self.extracted_model[pos][level] = self.model[pos][level]
        elif(leve == 0):
            self.extracted_model[pos] = {}
        elif(self.model[pos][leve]=="g" and leve in self.extracted_model[pos]):
            self.extracted_model[pos].pop(leve)
        ## Make Positions on Top of the Max Green
        for lev in range(leve+1, 6):
            if(lev in self.extracted_model[pos]):
                self.extracted_model[pos].pop(lev)

    def extract_model(self):
        """
        Function extract_model, to extract the existant blocks only.
        ---
        Paramrters:
        @param: None
        ---
        @return: None
        """
        for pos in self.model:
            for level in self.model[pos]:
                ## If the key exists in the extracted model
                if (level in self.extracted_model[pos]):
                    if(self.extracted_model[pos][level]!=self.model[pos][level] and self.model[pos][level]!="g"):
                        if self.latcher >= 10:
                            # print("\nUnLatched", pos, level)
                            self.copy_model(pos, level)
                            self.latcher = 0
                        else:
                            self.latcher += 2
                            # print("\nLatched", pos, level)
                    else:
                        self.copy_model(pos, level)
                        # print("\nSame Color Or Green", pos, level)
                else:
                    self.copy_model(pos, level)
                    # print("\n New Color", pos, level)
        self.latcher -= 1
        
    def model_accumulator(self, data):
        """
        Function model_accumulator, to accumulate vision info into a model.
        ---
        Paramrters:
        @param: data, String, discrete vision representation.
        ---
        @return: None
        """
        ## Read Vision Data
        self.vision_dict = json.loads(data.data)
        ## Reset Model
        self.model = {
            "p_01_08" : {1 : "g", 2 : "g", 3 : "g", 4 : "g", 5 : "g"},
            "p_01_09" : {1 : "g", 2 : "g", 3 : "g", 4 : "g", 5 : "g"},
            "p_01_10" : {1 : "g", 2 : "g", 3 : "g", 4 : "g", 5 : "g"},
            "p_01_11" : {1 : "g", 2 : "g", 3 : "g", 4 : "g", 5 : "g"},
            "p_01_12" : {1 : "g", 2 : "g", 3 : "g", 4 : "g", 5 : "g"},
            "p_01_13" : {1 : "g", 2 : "g", 3 : "g", 4 : "g", 5 : "g"},
            "p_01_14" : {1 : "g", 2 : "g", 3 : "g", 4 : "g", 5 : "g"},
            "p_02_07" : {1 : "g", 2 : "g", 3 : "g", 4 : "g", 5 : "g"},
            "p_02_08" : {1 : "g", 2 : "g", 3 : "g", 4 : "g", 5 : "g"},
            "p_02_09" : {1 : "g", 2 : "g", 3 : "g", 4 : "g", 5 : "g"},
            "p_02_10" : {1 : "g", 2 : "g", 3 : "g", 4 : "g", 5 : "g"},
            "p_02_11" : {1 : "g", 2 : "g", 3 : "g", 4 : "g", 5 : "g"},
            "p_02_12" : {1 : "g", 2 : "g", 3 : "g", 4 : "g", 5 : "g"},
            "p_02_13" : {1 : "g", 2 : "g", 3 : "g", 4 : "g", 5 : "g"},
            "p_02_14" : {1 : "g", 2 : "g", 3 : "g", 4 : "g", 5 : "g"},
            "p_02_15" : {1 : "g", 2 : "g", 3 : "g", 4 : "g", 5 : "g"},
            "p_03_07" : {1 : "g", 2 : "g", 3 : "g", 4 : "g", 5 : "g"},
            "p_03_08" : {1 : "g", 2 : "g", 3 : "g", 4 : "g", 5 : "g"},
            "p_03_09" : {1 : "g", 2 : "g", 3 : "g", 4 : "g", 5 : "g"},
            "p_03_10" : {1 : "g", 2 : "g", 3 : "g", 4 : "g", 5 : "g"},
            "p_03_11" : {1 : "g", 2 : "g", 3 : "g", 4 : "g", 5 : "g"},
            "p_03_12" : {1 : "g", 2 : "g", 3 : "g", 4 : "g", 5 : "g"},
            "p_03_13" : {1 : "g", 2 : "g", 3 : "g", 4 : "g", 5 : "g"},
            "p_03_14" : {1 : "g", 2 : "g", 3 : "g", 4 : "g", 5 : "g"},
            "p_03_15" : {1 : "g", 2 : "g", 3 : "g", 4 : "g", 5 : "g"},
            "p_04_07" : {1 : "g", 2 : "g", 3 : "g", 4 : "g", 5 : "g"},
            "p_04_08" : {1 : "g", 2 : "g", 3 : "g", 4 : "g", 5 : "g"},
            "p_04_09" : {1 : "g", 2 : "g", 3 : "g", 4 : "g", 5 : "g"},
            "p_04_10" : {1 : "g", 2 : "g", 3 : "g", 4 : "g", 5 : "g"},
            "p_04_11" : {1 : "g", 2 : "g", 3 : "g", 4 : "g", 5 : "g"},
            "p_04_12" : {1 : "g", 2 : "g", 3 : "g", 4 : "g", 5 : "g"},
            "p_04_13" : {1 : "g", 2 : "g", 3 : "g", 4 : "g", 5 : "g"},
            "p_04_14" : {1 : "g", 2 : "g", 3 : "g", 4 : "g", 5 : "g"},
            "p_04_15" : {1 : "g", 2 : "g", 3 : "g", 4 : "g", 5 : "g"},
            "p_05_07" : {1 : "g", 2 : "g", 3 : "g", 4 : "g", 5 : "g"},
            "p_05_08" : {1 : "g", 2 : "g", 3 : "g", 4 : "g", 5 : "g"},
            "p_05_09" : {1 : "g", 2 : "g", 3 : "g", 4 : "g", 5 : "g"},
            "p_05_10" : {1 : "g", 2 : "g", 3 : "g", 4 : "g", 5 : "g"},
            "p_05_11" : {1 : "g", 2 : "g", 3 : "g", 4 : "g", 5 : "g"},
            "p_05_12" : {1 : "g", 2 : "g", 3 : "g", 4 : "g", 5 : "g"},
            "p_05_13" : {1 : "g", 2 : "g", 3 : "g", 4 : "g", 5 : "g"},
            "p_05_14" : {1 : "g", 2 : "g", 3 : "g", 4 : "g", 5 : "g"},
            "p_05_15" : {1 : "g", 2 : "g", 3 : "g", 4 : "g", 5 : "g"},
            "p_06_07" : {1 : "g", 2 : "g", 3 : "g", 4 : "g", 5 : "g"},
            "p_06_08" : {1 : "g", 2 : "g", 3 : "g", 4 : "g", 5 : "g"},
            "p_06_09" : {1 : "g", 2 : "g", 3 : "g", 4 : "g", 5 : "g"},
            "p_06_10" : {1 : "g", 2 : "g", 3 : "g", 4 : "g", 5 : "g"},
            "p_06_11" : {1 : "g", 2 : "g", 3 : "g", 4 : "g", 5 : "g"},
            "p_06_12" : {1 : "g", 2 : "g", 3 : "g", 4 : "g", 5 : "g"},
            "p_06_13" : {1 : "g", 2 : "g", 3 : "g", 4 : "g", 5 : "g"},
            "p_06_14" : {1 : "g", 2 : "g", 3 : "g", 4 : "g", 5 : "g"},
            "p_06_15" : {1 : "g", 2 : "g", 3 : "g", 4 : "g", 5 : "g"},
            "p_07_08" : {1 : "g", 2 : "g", 3 : "g", 4 : "g", 5 : "g"},
            "p_07_09" : {1 : "g", 2 : "g", 3 : "g", 4 : "g", 5 : "g"},
            "p_07_10" : {1 : "g", 2 : "g", 3 : "g", 4 : "g", 5 : "g"},
            "p_07_11" : {1 : "g", 2 : "g", 3 : "g", 4 : "g", 5 : "g"},
            "p_07_12" : {1 : "g", 2 : "g", 3 : "g", 4 : "g", 5 : "g"},
            "p_07_13" : {1 : "g", 2 : "g", 3 : "g", 4 : "g", 5 : "g"},
            "p_07_14" : {1 : "g", 2 : "g", 3 : "g", 4 : "g", 5 : "g"}
        }
        ## Fill model
        for pos in self.model:
            self.model[pos][int(self.vision_dict[pos][1])] = self.vision_dict[pos][0]
        self.extract_model()
        self.dumpData()
        fin_model = json.dumps(self.extracted_model)
        self.model_pub.publish(fin_model)
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
        rospy.init_node('model_accumulator', anonymous=True)
        self.model_pub = rospy.Publisher("model", String, queue_size=10)
        self.rate = rospy.Rate(10) # 10hz
        rospy.Subscriber('vision_dict', String, self.model_accumulator)
        rospy.sleep(1)
        rospy.spin()
##===============================================================================

if __name__ == '__main__':
    vision_iface_ = ModelAcumulator(sys.argv[1])
    vision_iface_.receiver()
