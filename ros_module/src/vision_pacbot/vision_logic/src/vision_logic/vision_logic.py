#!/usr/bin/python3.8
# -*- coding: utf-8 -*-
"""
| author:
| Belal HMEDAN, 
| LIG lab/ Marvin Team, 
| France, 2023.
| VISON Logic.

| 2nd Author - Code Modified for Web Interface to operate the robot:
| Pardeep Kumar, 
| LIG lab/ Marvin Team, 
| France, 2023.
| VISON Logic.
"""

import rospy
from std_msgs.msg import String
import sys
import json
import copy

# ------- pardeep's variables starts
legotype = {}
legomoved = {}
### ------ ends---------------------

##=====================
## Vision Logic Class |
##=====================
class VisionLogic:
    def __init__(self, path, model):
        """ """
        self.path = path
        self.model = model
        self.picked = {}
        self.placed = {}
        self.picked_colors = {"r": 0, "b": 0, "y": 0, "w": 0, "l": 0, "o": 0}
        self.old_vision_dict = {}
        self.vision_dict = None
        self.state = {}
        self.lego_map = {}
        self.occupied_positions = []
        self.loadData()
        self.depth_threshold = 39
        if(model=="simple"):
            self.depth_threshold = 19
        # pardeep's variable
        self.lego_data=[]

    def loadData(self):
        """
        Function: loadData, to load Legos position/orientation and past state.
        ---
        Parameters:
        @param: None
        ---
        @return: None
        """
        with open(self.path + "state.json", "r") as fp:
            self.state = json.load(fp)
        with open(self.path + "lego_map.json", "r") as fp:
            self.lego_map = json.load(fp)
        # pardeep's modifications 

    def dumpData(self):
        """
        Function: dumpData, to store Legos position/orientation and past state into JSON.
        ---
        Parameters:
        @param: None
        ---
        @return: None
        """
        with open(self.path + "state.json", "w") as fp:
            json.dump(self.state, fp)
        with open(self.path + "lego_map.json", "w") as fp:
            json.dump(self.lego_map, fp)

    def fillOccupiedPositions(self):
        """
        Function: fillOccupiedPositions, to construct a list of the positions occupied by Legos.
        ---
        Parameters:
        @param: None
        ---
        @return: None
        """
        self.occupied_positions = []

        for lego in self.lego_map[self.model]:
            if lego[1] == "c":
                pos = self.lego_map[self.model][lego][0]
                if not (pos in self.occupied_positions):
                    self.occupied_positions.append(pos)
            else:
                p1 = self.lego_map[self.model][lego][0]
                p2 = self.lego_map[self.model][lego][1]
                if not (p1 in self.occupied_positions):
                    self.occupied_positions.append(p1)
                if not (p2 in self.occupied_positions):
                    self.occupied_positions.append(p2)
    # ##### ---------- pardeep's publisher starts --------------#
    # def legodataReader(self):
    #     """
    #     Function: legodataReader, to read the information of lego blocks for actions in web applications
    #     ---
    #     Parameters:
    #     @param: None
    #     ---
    #     @return: None
    #     """
    #     rospy.init_node("legodatanode", anonymous=True)
    #     self.lego_data_pub = rospy.Publisher("lego_data", String, queue_size=10)
    #     self.rate = rospy.Rate(10)  # 10hz
    #     # rospy.Subscriber("vision_dict", String, self.pickPlaceFinder)
    #     rospy.sleep(1)
    #     rospy.spin()
    # # ##### ---------- pardeep's publisher ends --------------#
    def visionReader(self):
        """
        Function: visionReader, to read the discrete vision information.
        ---
        Parameters:
        @param: None
        ---
        @return: None
        """
        rospy.init_node("vision_rational", anonymous=True)
        self.lego_map_pub = rospy.Publisher("lego_map", String, queue_size=10)
        # pardeep's data publisher
        self.lego_data_pub = rospy.Publisher("lego_data", String, queue_size=10)
        self.rate = rospy.Rate(10)  # 10hz
        rospy.Subscriber("vision_dict", String, self.pickPlaceFinder)
        rospy.sleep(1)
        rospy.spin()

    def key2pos(self, s):
        """
        Function: key2pos, to obtain x, y cell coordinates from string of shape
        'p_xx_yy'. --- Parameters: @param: s, string of shape 'p_xx_yy'. ---
        @return: tuple, (x, y), the cell coordinates.
        """
        o = s.split("_")
        return [int(o[1]), int(o[2])]

    def getBlockByPos(self, pos):
        """
        Function: getBlockByPos, get a list of the legos depending on their p_xx_yy position.
        ---
        Parameters:
        @param: pos, string, p_xx_yy, position.
        ---
        @return: lego_xx_yy, list of strings, all legos that belong to this position p_xx_yy.
        """
        lego_xx_yy = []
        for lego in self.lego_map[self.model]:
            if pos in self.lego_map[self.model][lego]:
                lego_xx_yy.append(lego)
        return lego_xx_yy

    def getMaxZ(self, lego_xx_yy):
        """
        Function: getMaxZ, get the top lego in a list of legos shares same xx_yy position.
        ---
        Parameters:
        @param: lego_xx_yy, list of strings, all legos that belong to this position p_xx_yy.
        ---
        @return: lego_max, max_z, (string, the maximum lego; int, maximum Z).
        """
        max_z = 0
        lego_max = lego_xx_yy[0]
        for lego in lego_xx_yy:
            if (lego[1] == "c") and (self.lego_map[self.model][lego][1] > max_z):
                max_z = self.lego_map[self.model][lego][1]
                lego_max = lego
            elif (lego[1] == "b") and (self.lego_map[self.model][lego][2] > max_z):
                max_z = self.lego_map[self.model][lego][2]
                lego_max = lego
        return lego_max, max_z

    def neighbours(self, pos):
        """
        Function: neighbours, to find the nifgbouring position, and the rotation for a lego Brick.
        ---
        Parameters:
        @param: pos, string, p_xx_yy, position.
        ---
        @return: None
        """
        x0, y0 = self.key2pos(pos)

        p1 = "p_" + f"{x0+1:02}" + "_" + f"{y0:02}"
        p2 = "p_" + f"{x0-1:02}" + "_" + f"{y0:02}"
        p3 = "p_" + f"{x0:02}" + "_" + f"{y0+1:02}"
        p4 = "p_" + f"{x0:02}" + "_" + f"{y0-1:02}"

        if p1 in self.placed:
            if self.placed[p1] == self.placed[pos]:
                return [pos, p1, True]
        if p2 in self.placed:
            if self.placed[p2] == self.placed[pos]:
                return [p2, pos, True]
        if p3 in self.placed:
            if self.placed[p3] == self.placed[pos]:
                return [pos, p3, False]
        if p4 in self.placed:
            if self.placed[p4] == self.placed[pos]:
                return [p4, pos, False]

        if self.getBlockByPos(p1):
            lego, _ = self.getMaxZ(self.getBlockByPos(p1))
            if (lego[1] == "b") and (self.vision_dict[p1] == self.placed[pos]):
                return [pos, p1, True]
        if self.getBlockByPos(p2):
            lego, _ = self.getMaxZ(self.getBlockByPos(p2))
            if (lego[1] == "b") and (self.vision_dict[p2] == self.placed[pos]):
                return [p2, pos, True]
        if self.getBlockByPos(p3):
            lego, _ = self.getMaxZ(self.getBlockByPos(p3))
            if (lego[1] == "b") and (self.vision_dict[p3] == self.placed[pos]):
                return [pos, p3, False]
        if self.getBlockByPos(p4):
            lego, _ = self.getMaxZ(self.getBlockByPos(p4))
            if (lego[1] == "b") and (self.vision_dict[p4] == self.placed[pos]):
                return [p4, pos, False]

        return False

    def updateLegoMap(self, lego, pos, rot=False):
        """
        Function: updateLegoMap, to update the lego_map (maps legos to positions).
        ---
        Parameters:
        @param: lego, string, i.e: yb1, y --> yellow color, b/c --> brick/cube
        @param: pos, string/List of two strings cube/brick, p_xx_yy, [p_xx1_yy1, p_xx2_yy2] positions.
        @param: depth, int, to represent the placement depth of the lego.
        @param: rot, rotation of the lego brick.
        ---
        @return: None
        """
        # Cube
        if(lego[1] =="c"):
            self.lego_map[self.model][lego] = [pos, self.vision_dict[pos][1]]
        # Brick
        elif(lego[1] =="b"):
            self.lego_map[self.model][lego] = [pos[0], pos[1], self.vision_dict[pos[0]][1], rot]

    def pickPlaceMatcher(self):
        global legotype
        """

        """
        self.placed_neighbours = []
        self.popped_lego_bricks = []
        # Loop the Bricks First
        for lego in list(self.picked):
            for place_pos in list(self.placed):
                if ((place_pos in self.placed_neighbours) or (lego in self.popped_lego_bricks)):
                    continue
                # Brick Same Color is Placed with neighbour
                if ((lego[0] == self.placed[place_pos][0]) and (lego[1] =="b") and (self.neighbours(place_pos))):
                    pos1, pos2, rot = self.neighbours(place_pos)
                    self.placed_neighbours.append(pos1)
                    self.placed_neighbours.append(pos2)
                    self.updateLegoMap(lego, [pos1, pos2], rot)
                    ppick1, ppick2 = self.picked[lego][0:2]
                    self.picked.pop(lego)
                    self.popped_lego_bricks.append(lego)
                    self.picked_colors[lego[0]] = self.picked_colors[lego[0]] - 1
                    rospy.logwarn("pickPlaceMatcher: Lego Brick ({}) was picked and placed in {}, {}".format(lego, pos1, pos2))
                    #global legotype
                    #global legomoved
                    legotype={'lego':lego}
                    # brick_data_pickplace.append(bdata)
                    if(pos1 in self.placed):
                        self.placed.pop(pos1)
                    if(pos2 in self.placed):
                        self.placed.pop(pos2)
                    self.state[self.model][ppick1] = self.vision_dict[ppick1]
                    self.state[self.model][ppick2] = self.vision_dict[ppick2]
                    self.state[self.model][pos1] = self.vision_dict[pos1]
                    self.state[self.model][pos2] = self.vision_dict[pos2]

        self.popped_lego_cubes = []
        # Loop the Cubes After
        for lego in list(self.picked):
            for place_pos in list(self.placed):
                if lego in self.popped_lego_cubes:
                    continue
                # Cube Same Color
                if ((lego[0] == self.placed[place_pos][0]) and (lego[1] =="c")):
                    self.updateLegoMap(lego, place_pos)
                    ppick = self.picked[lego][0]
                    self.popped_lego_cubes.append(lego)
                    self.picked.pop(lego)
                    self.placed.pop(place_pos)
                    self.picked_colors[lego[0]] = self.picked_colors[lego[0]] - 1
                    rospy.logwarn("pickPlaceMatcher: Lego Cube ({}) was picked and placed in {}".format(lego, place_pos))
                    #global legotype
                    #global legomoved
                    legotype={'lego':lego}
                    # cube_data_pickplace = cdata
                    self.state[self.model][ppick] = self.vision_dict[ppick]
                    self.state[self.model][place_pos] = self.vision_dict[place_pos]

        # if(not bool(self.placed)):
        #     self.placed = {}
        #     rospy.logwarn("pickPlaceMatcher: placed legos reset" )

    def pickPlaceFinder(self, data):
        global legomoved
        """
        Function: pickPlaceFinder
        """
        self.dumpData()
        self.vision_dict = json.loads(data.data)
        depth_counter = 0
        for p_ in self.vision_dict:
            depth_counter += int(self.vision_dict[p_][1])
            if ((int(self.vision_dict[p_][1])==0) and (self.vision_dict[p_][0]!='g')):
                self.vision_dict[p_][1] = 1
                # rospy.logwarn("pickPlaceFinder: Position {} depth was corrected!\n".format(p_))
                
        # rospy.logwarn("pickPlaceFinder: Depth Counter is {}".format(depth_counter))
        if ((self.vision_dict != self.old_vision_dict) and depth_counter == self.depth_threshold):
            self.old_vision_dict = self.vision_dict
            self.picked = {}
            self.placed = {}
            # Pick Loop
            # Loop Lego Map
            for lego in self.lego_map[self.model]:
                """
                # Get Position/s
                # Check if any of these positions if:
                # 1. depth has increased, or
                # 2. color has changed
                # If yes then:
                # 1. add the color to the picked _colors
                # 2. add the self.state[model] to the picked positions
                """
                # Color
                c = lego[0]
                # Cube
                if lego[1] == "c":
                    pos_ = self.lego_map[self.model][lego][0]
                    # information change (color/depth) at this position
                    if self.vision_dict[pos_] != self.state[self.model][pos_]:
                        if self.vision_dict[pos_][1] <= self.state[self.model][pos_][1]:
                            self.picked[lego] = [pos_, self.state[self.model][pos_]]
                            self.picked_colors[c] = self.picked_colors[c] + 1
                # Brick
                elif lego[1] == "b":
                    p1_ = self.lego_map[self.model][lego][0]
                    p2_ = self.lego_map[self.model][lego][1]
                    # information change (color/depth) at this position
                    if (
                        self.vision_dict[p1_] != self.state[self.model][p1_]
                        and self.vision_dict[p2_] != self.state[self.model][p2_]
                    ):
                        if (
                            int(self.vision_dict[p1_][1])
                            < int(self.state[self.model][p1_][1])
                            and int(self.vision_dict[p2_][1])
                            < int(self.state[self.model][p2_][1])
                        ) or (
                            (
                                int(self.vision_dict[p1_][1])
                                == int(self.state[self.model][p1_][1])
                                and int(self.vision_dict[p2_][1])
                                == int(self.state[self.model][p2_][1])
                            )
                        ):
                            self.picked[lego] = [p1_, p2_, self.state[self.model][p1_]]
                            self.picked_colors[c] = self.picked_colors[c] + 1

            # Place Loop
            # Loop the Vision Dictionary
            for pos in self.vision_dict:
                """
                # Get the changed positions from the self.state[model]
                # if depth has been decreased:
                #  add self.vision_dict[pos] to placed
                # if color has changed, and color in self.picked_colors:
                """
                if self.vision_dict[pos] != self.state[self.model][pos]:
                    if (
                        int(self.vision_dict[pos][1]) - int(self.state[self.model][pos][1])
                        == 1
                    ) and (self.picked_colors[self.vision_dict[pos][0]] > 0):
                        self.placed[pos] = self.vision_dict[pos]
                    elif (
                        int(self.vision_dict[pos][1]) - int(self.state[self.model][pos][1])
                        == 0
                    ) and (self.picked_colors[self.vision_dict[pos][0]] > 0):
                        self.placed[pos] = self.vision_dict[pos]
            rospy.logwarn("\n pickPlaceFinder \n")
            rospy.logwarn("\n--------Picked--------\n")
            rospy.logwarn(str(self.picked))
            rospy.logwarn("\n--------Placed--------\n")
            rospy.logwarn(str(self.placed))
            rospy.logwarn("\n======================\n")
            #global legomoved
            legomoved={'pickedfrom':str(self.picked), 'placedat':str(self.placed)}

            self.pickPlaceMatcher()
            self.fillOccupiedPositions()

        msg_to_pub = copy.deepcopy(self.lego_map[self.model])

        for lego in self.picked:
            rospy.logwarn("pickPlaceFinder: Lego ( {} ) was picked, but not placed!".format(lego))
            del msg_to_pub[lego]

        lego_map = json.dumps(msg_to_pub)
        self.lego_map_pub.publish(lego_map)
        ## ------ pardeep's lego_data publisher
        p_lego_data = {'lego_data':legotype, 'lego_moved':legomoved}
        # brick_data_pickplace.clear()
        # cube_data_pickplace.clear()
        # lego_data = json.dumps(jlego_data)
        self.lego_data_pub.publish(str(p_lego_data))
        self.rate.sleep()

        # if bool(self.picked) or bool(self.placed):
        #     print("\n &&&lego_map&&&\n")
        #     print("\n", self.lego_map[self.model])
        #     print("\n &&&picked&&&\n")
        #     print(self.picked)
        #     print("\n &&&placed&&&\n")
        #     print(self.placed)
        #     print("\n &&&occupied_positions&&&\n")
        #     print(self.occupied_positions)
        #     print("\n &&&vision_dict&&&\n")
        #     print(self.vision_dict)
        #     print("\n &&&state&&&\n")
        #     print(self.state[self.model])
        #     print("\n ^^^^^^^^^^^^^^^^^\nEnd\n")
##=============================================
if __name__ == "__main__":
    try:
        vl_ = VisionLogic(sys.argv[1], sys.argv[2])
        vl_.visionReader()
        # vl_.legodataReader()

    except rospy.ROSInterruptException:
        rospy.logerr("Error in the Vision Logic")
