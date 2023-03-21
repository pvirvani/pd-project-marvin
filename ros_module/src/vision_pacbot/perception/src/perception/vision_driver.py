#! /usr/bin/env python3.8
# -*- coding: utf-8 -*-
"""
| author:
| Belal HMEDAN, 
| LIG lab/ Marvin Team, 
| France, 2022.
| image processing node publisher.
"""

import rospy
from std_msgs.msg import String, Bool
import json
import time
import numpy as np
import pyrealsense2.pyrealsense2 as rs
from rgb_handler import RGBProcessor
from depth_handler import DepthProcessor

class VisionDriver:

    def __init__(self):
        """
        Class VisionDriver: Driver to do scene segmentation using 
          Realsense L515 camera and ROS.
        ---
        Parameters:
        @param: None.
        """
        self.rgbP_   = RGBProcessor()
        self.depthP_ = DepthProcessor()

        self.frame_idx = 0
        self.latch = np.ones((11, 23, 3), np.uint8)

        self.init_cam()

        self.vision_dict = {
            'p_00_00':[ 'g', 0 ], 'p_00_01':[ 'g', 0 ], 'p_00_02':[ 'g', 0 ], 'p_00_03':[ 'g', 0 ], 'p_00_04':[ 'g', 0 ], 'p_00_05':[ 'g', 0 ], 'p_00_06':[ 'g', 0 ], 'p_00_07':[ 'g', 0 ], 'p_00_08':[ 'g', 0 ], 'p_00_09':[ 'g', 0 ], 'p_00_10':[ 'g', 0 ], 'p_00_11':[ 'g', 0 ], 'p_00_12':[ 'g', 0 ], 'p_00_13':[ 'g', 0 ], 'p_00_14':[ 'g', 0 ], 'p_00_15':[ 'g', 0 ], 'p_00_16':[ 'g', 0 ], 'p_00_17':[ 'g', 0 ], 'p_00_18':[ 'g', 0 ], 'p_00_19':[ 'g', 0 ], 'p_00_20':[ 'g', 0 ], 'p_00_21':[ 'g', 0 ], 'p_00_22':[ 'g', 0 ],
            'p_01_00':[ 'g', 0 ], 'p_01_01':[ 'g', 0 ], 'p_01_02':[ 'g', 0 ], 'p_01_03':[ 'g', 0 ], 'p_01_04':[ 'g', 0 ], 'p_01_05':[ 'g', 0 ], 'p_01_06':[ 'g', 0 ], 'p_01_07':[ 'g', 1 ], 'p_01_08':[ 'g', 0 ], 'p_01_09':[ 'g', 0 ], 'p_01_10':[ 'g', 0 ], 'p_01_11':[ 'g', 0 ], 'p_01_12':[ 'g', 0 ], 'p_01_13':[ 'g', 0 ], 'p_01_14':[ 'g', 0 ], 'p_01_15':[ 'g', 1 ], 'p_01_16':[ 'g', 0 ], 'p_01_17':[ 'g', 0 ], 'p_01_18':[ 'g', 0 ], 'p_01_19':[ 'g', 0 ], 'p_01_20':[ 'g', 0 ], 'p_01_21':[ 'g', 0 ], 'p_01_22':[ 'g', 0 ],
            'p_02_00':[ 'g', 0 ], 'p_02_01':[ 'g', 0 ], 'p_02_02':[ 'g', 0 ], 'p_02_03':[ 'g', 0 ], 'p_02_04':[ 'g', 0 ], 'p_02_05':[ 'g', 0 ], 'p_02_06':[ 'g', 0 ], 'p_02_07':[ 'g', 0 ], 'p_02_08':[ 'g', 0 ], 'p_02_09':[ 'g', 0 ], 'p_02_10':[ 'g', 0 ], 'p_02_11':[ 'g', 0 ], 'p_02_12':[ 'g', 0 ], 'p_02_13':[ 'g', 0 ], 'p_02_14':[ 'g', 0 ], 'p_02_15':[ 'g', 0 ], 'p_02_16':[ 'g', 0 ], 'p_02_17':[ 'g', 0 ], 'p_02_18':[ 'g', 0 ], 'p_02_19':[ 'g', 0 ], 'p_02_20':[ 'g', 0 ], 'p_02_21':[ 'g', 0 ], 'p_02_22':[ 'g', 0 ],
            'p_03_00':[ 'g', 0 ], 'p_03_01':[ 'g', 0 ], 'p_03_02':[ 'g', 0 ], 'p_03_03':[ 'g', 0 ], 'p_03_04':[ 'g', 0 ], 'p_03_05':[ 'g', 0 ], 'p_03_06':[ 'g', 0 ], 'p_03_07':[ 'g', 0 ], 'p_03_08':[ 'g', 0 ], 'p_03_09':[ 'g', 0 ], 'p_03_10':[ 'g', 0 ], 'p_03_11':[ 'g', 0 ], 'p_03_12':[ 'g', 0 ], 'p_03_13':[ 'g', 0 ], 'p_03_14':[ 'g', 0 ], 'p_03_15':[ 'g', 0 ], 'p_03_16':[ 'g', 0 ], 'p_03_17':[ 'g', 0 ], 'p_03_18':[ 'g', 0 ], 'p_03_19':[ 'g', 0 ], 'p_03_20':[ 'g', 0 ], 'p_03_21':[ 'g', 0 ], 'p_03_22':[ 'g', 0 ],
            'p_04_00':[ 'g', 0 ], 'p_04_01':[ 'g', 0 ], 'p_04_02':[ 'g', 0 ], 'p_04_03':[ 'g', 0 ], 'p_04_04':[ 'g', 0 ], 'p_04_05':[ 'g', 0 ], 'p_04_06':[ 'g', 0 ], 'p_04_07':[ 'g', 0 ], 'p_04_08':[ 'g', 0 ], 'p_04_09':[ 'g', 0 ], 'p_04_10':[ 'g', 0 ], 'p_04_11':[ 'g', 0 ], 'p_04_12':[ 'g', 0 ], 'p_04_13':[ 'g', 0 ], 'p_04_14':[ 'g', 0 ], 'p_04_15':[ 'g', 0 ], 'p_04_16':[ 'g', 0 ], 'p_04_17':[ 'g', 0 ], 'p_04_18':[ 'g', 0 ], 'p_04_19':[ 'g', 0 ], 'p_04_20':[ 'g', 0 ], 'p_04_21':[ 'g', 0 ], 'p_04_22':[ 'g', 0 ],
            'p_05_00':[ 'g', 0 ], 'p_05_01':[ 'g', 0 ], 'p_05_02':[ 'g', 0 ], 'p_05_03':[ 'g', 0 ], 'p_05_04':[ 'g', 0 ], 'p_05_05':[ 'g', 0 ], 'p_05_06':[ 'g', 0 ], 'p_05_07':[ 'g', 0 ], 'p_05_08':[ 'g', 0 ], 'p_05_09':[ 'g', 0 ], 'p_05_10':[ 'g', 0 ], 'p_05_11':[ 'g', 0 ], 'p_05_12':[ 'g', 0 ], 'p_05_13':[ 'g', 0 ], 'p_05_14':[ 'g', 0 ], 'p_05_15':[ 'g', 0 ], 'p_05_16':[ 'g', 0 ], 'p_05_17':[ 'g', 0 ], 'p_05_18':[ 'g', 0 ], 'p_05_19':[ 'g', 0 ], 'p_05_20':[ 'g', 0 ], 'p_05_21':[ 'g', 0 ], 'p_05_22':[ 'g', 0 ],
            'p_06_00':[ 'g', 0 ], 'p_06_01':[ 'g', 0 ], 'p_06_02':[ 'g', 0 ], 'p_06_03':[ 'g', 0 ], 'p_06_04':[ 'g', 0 ], 'p_06_05':[ 'g', 0 ], 'p_06_06':[ 'g', 0 ], 'p_06_07':[ 'g', 0 ], 'p_06_08':[ 'g', 0 ], 'p_06_09':[ 'g', 0 ], 'p_06_10':[ 'g', 0 ], 'p_06_11':[ 'g', 0 ], 'p_06_12':[ 'g', 0 ], 'p_06_13':[ 'g', 0 ], 'p_06_14':[ 'g', 0 ], 'p_06_15':[ 'g', 0 ], 'p_06_16':[ 'g', 0 ], 'p_06_17':[ 'g', 0 ], 'p_06_18':[ 'g', 0 ], 'p_06_19':[ 'g', 0 ], 'p_06_20':[ 'g', 0 ], 'p_06_21':[ 'g', 0 ], 'p_06_22':[ 'g', 0 ],
            'p_07_00':[ 'g', 0 ], 'p_07_01':[ 'g', 0 ], 'p_07_02':[ 'g', 0 ], 'p_07_03':[ 'g', 0 ], 'p_07_04':[ 'g', 0 ], 'p_07_05':[ 'g', 0 ], 'p_07_06':[ 'g', 0 ], 'p_07_07':[ 'g', 1 ], 'p_07_08':[ 'g', 0 ], 'p_07_09':[ 'g', 0 ], 'p_07_10':[ 'g', 0 ], 'p_07_11':[ 'g', 0 ], 'p_07_12':[ 'g', 0 ], 'p_07_13':[ 'g', 0 ], 'p_07_14':[ 'g', 0 ], 'p_07_15':[ 'g', 1 ], 'p_07_16':[ 'g', 0 ], 'p_07_17':[ 'g', 0 ], 'p_07_18':[ 'g', 0 ], 'p_07_19':[ 'g', 0 ], 'p_07_20':[ 'g', 0 ], 'p_07_21':[ 'g', 0 ], 'p_07_22':[ 'g', 0 ],
            'p_08_00':[ 'g', 0 ], 'p_08_01':[ 'g', 0 ], 'p_08_02':[ 'g', 0 ], 'p_08_03':[ 'g', 0 ], 'p_08_04':[ 'g', 0 ], 'p_08_05':[ 'g', 0 ], 'p_08_06':[ 'g', 0 ], 'p_08_07':[ 'g', 0 ], 'p_08_08':[ 'g', 0 ], 'p_08_09':[ 'g', 0 ], 'p_08_10':[ 'g', 0 ], 'p_08_11':[ 'g', 0 ], 'p_08_12':[ 'g', 0 ], 'p_08_13':[ 'g', 0 ], 'p_08_14':[ 'g', 0 ], 'p_08_15':[ 'g', 0 ], 'p_08_16':[ 'g', 0 ], 'p_08_17':[ 'g', 0 ], 'p_08_18':[ 'g', 0 ], 'p_08_19':[ 'g', 0 ], 'p_08_20':[ 'g', 0 ], 'p_08_21':[ 'g', 0 ], 'p_08_22':[ 'g', 0 ],
            'p_09_00':[ 'g', 0 ], 'p_09_01':[ 'g', 0 ], 'p_09_02':[ 'g', 0 ], 'p_09_03':[ 'g', 0 ], 'p_09_04':[ 'g', 0 ], 'p_09_05':[ 'g', 0 ], 'p_09_06':[ 'g', 0 ], 'p_09_07':[ 'g', 0 ], 'p_09_08':[ 'g', 0 ], 'p_09_09':[ 'g', 0 ], 'p_09_10':[ 'g', 0 ], 'p_09_11':[ 'g', 0 ], 'p_09_12':[ 'g', 0 ], 'p_09_13':[ 'g', 0 ], 'p_09_14':[ 'g', 0 ], 'p_09_15':[ 'g', 0 ], 'p_09_16':[ 'g', 0 ], 'p_09_17':[ 'g', 0 ], 'p_09_18':[ 'g', 0 ], 'p_09_19':[ 'g', 0 ], 'p_09_20':[ 'g', 0 ], 'p_09_21':[ 'g', 0 ], 'p_09_22':[ 'g', 0 ],
            'p_10_00':[ 'g', 0 ], 'p_10_01':[ 'g', 0 ], 'p_10_02':[ 'g', 0 ], 'p_10_03':[ 'g', 0 ], 'p_10_04':[ 'g', 0 ], 'p_10_05':[ 'g', 0 ], 'p_10_06':[ 'g', 0 ], 'p_10_07':[ 'g', 0 ], 'p_10_08':[ 'g', 0 ], 'p_10_09':[ 'g', 0 ], 'p_10_10':[ 'g', 0 ], 'p_10_11':[ 'g', 0 ], 'p_10_12':[ 'g', 0 ], 'p_10_13':[ 'g', 0 ], 'p_10_14':[ 'g', 0 ], 'p_10_15':[ 'g', 0 ], 'p_10_16':[ 'g', 0 ], 'p_10_17':[ 'g', 0 ], 'p_10_18':[ 'g', 0 ], 'p_10_19':[ 'g', 0 ], 'p_10_20':[ 'g', 0 ], 'p_10_21':[ 'g', 0 ], 'p_10_22':[ 'g', 0 ]
        }

    def init_cam(self):
        """
        Function: init_cam, to set the Realsense L515 camera parameters.
        ---
        Parameters:
        @param: None
        ---
        @return None.
        """
        # Start reading frames
        self.pipeline = rs.pipeline()
        self.config = rs.config()
        # RGB Stream
        self.config.enable_stream(rs.stream.color, 960, 540, rs.format.bgr8, 30)
        self.config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
        #
        self.profile = self.config.resolve(self.pipeline)

        # Start streaming
        self.pipeline.start(self.config)

        # Declare sensor object and set options
        depth_sensor = self.profile.get_device().first_depth_sensor()
        depth_sensor.set_option(rs.option.visual_preset, 5) # 5 is short range, 3 is low ambient light
        depth_sensor.set_option(rs.option.receiver_gain, 8)
        depth_sensor.set_option(rs.option.pre_processing_sharpening, 0.0)
        depth_sensor.set_option(rs.option.post_processing_sharpening, 3.0)
        depth_sensor.set_option(rs.option.laser_power, 100)
        depth_sensor.set_option(rs.option.confidence_threshold, 1)
        depth_sensor.set_option(rs.option.digital_gain, 2.0)
        depth_sensor.set_option(rs.option.noise_filtering, 4)
        # Get the sensor once at the beginning. (Sensor index: 1)
        sensor = self.pipeline.get_active_profile().get_device().query_sensors()[1]

        # Set the exposure anytime during the operation
        sensor.set_option(rs.option.exposure, 800.000)
        sensor.set_option(rs.option.gain, 0.000)

        # # Filters
        self.threshold_filter = rs.threshold_filter(min_dist=0.8, max_dist=1.25)
        self.temporal_filter  = rs.temporal_filter(smooth_alpha=0.1, smooth_delta = 90.0, persistence_control=0)
        
        # # Colorizer
        self.colorizer = rs.colorizer()
        self.colorizer.set_option(rs.option.visual_preset, 1) # 0=Dynamic, 1=Fixed, 2=Near, 3=Far
        self.colorizer.set_option(rs.option.min_distance, 0.8)
        self.colorizer.set_option(rs.option.max_distance, 1.25)

        # # Align element
        align_to = rs.stream.color
        self.align = rs.align(align_to)

    def init_ros(self):
        """
        Function: init_ros, to set the ROS node/publishers/subscriber.
        ---
        Parameters:
        @param: None
        ---
        @return None.
        """
        self.dict_pub = rospy.Publisher("vision_dict", String, queue_size=10)
        self.hand_pub = rospy.Publisher("hand_detection", Bool, queue_size=10)
        rospy.Subscriber("yumi_motion_status", Bool, self.motionCallback)
        rospy.init_node("perception", anonymous=True)
        self.rate = rospy.Rate(10)  # 10hz

    def motionCallback(self, data):
        """
        Function: motionCallback, to assign the lock the vision in case of robot motion.
        ---
        Parameters:
        @param: None

        ---
        @return None.
        """
        self.rgbP_.motion_detected   = data.data
        self.depthP_.motion_detected = data.data
    
    def run(self, colorizer=False, lock_delay=1):
        """
        Function: run, to run the vision system.
        ---
        Parameters:
        @param: colorizer, bool, to colorize the depth.
        @param: lock_delay, int, time delay for hand or motion locks.

        ---
        @return None.
        """
        skipper = 0
        
        while not rospy.is_shutdown():
            skipper += 1
            frames = self.pipeline.wait_for_frames()
            if skipper < 5:
                continue
            else:
                skipper -= 1

            # Align Frames
            frames = self.align.process(frames)

            color_frame = frames.get_color_frame()
            depth_frame = frames.get_depth_frame()
            if (not depth_frame) or (not color_frame):
                continue

            # Depth
            depth_frame = self.threshold_filter.process(depth_frame)
            depth_frame = self.temporal_filter.process(depth_frame)

            # Convert images to numpy arrays
            color_array = np.asanyarray(color_frame.get_data())
            if colorizer:
                depth_array = np.asanyarray(colorizer.colorize(depth_frame).get_data())
            else:
                depth_array = np.asanyarray(depth_frame.get_data())


            ################
            ## Processing ##
            ################
            self.rgbP_.get_rgb_info(color_array)
            self.depthP_.hand = self.rgbP_.hand
            self.hand_pub.publish(self.rgbP_.hand)

            self.depthP_.get_depth_info(depth_array, self.rgbP_.greenRect)

            # Hand Detected!
            if self.rgbP_.hand:
                rospy.logwarn("\nHand/Arm Detected, sleeping {} Seconds ...\n".format(lock_delay))
                time.sleep(lock_delay)
                continue
            if self.rgbP_.motion_detected:
                rospy.logwarn("\nMotion Detected, sleeping {} Seconds ...\n".format(lock_delay))
                time.sleep(lock_delay)
                continue

            corners = ["p_01_07", "p_01_15", "p_07_07", "p_07_15"]
            self.frame_idx += 1

            for k in self.vision_dict:
                # Color
                self.vision_dict[k][0] = self.rgbP_.cellsState[k]
                
                x, y = self.depthP_.key2pos(k)
                
                self.latch[x, y, self.frame_idx-1] = int(self.depthP_.cellsState[k])
                
                ## Correct Depth from Color
                if(self.vision_dict[k][0] == 'g'):
                    self.latch[x, y] = (0, 0, 0)

                if(self.vision_dict[k][1] == 0 and self.vision_dict[k][0] != 'g'):
                    self.latch[x, y] = (1, 1, 1)
                                   
                ## Green Corners
                if (k in corners):
                    self.latch[x, y] = (1, 1, 1)
                
                ## Latch Memory
                if (self.latch[x, y, 0] == self.latch[x, y, 1]) and (self.latch[x, y, 0] == self.latch[x, y, 2]):
                    self.vision_dict[k][1] = int(self.latch[x, y, 0])

            my_dict = json.dumps(self.vision_dict)
            
            ## Reset Sequence Counter
            if(self.frame_idx % 3 == 0):
                self.frame_idx = 0

            ## Publish vision Data
            self.dict_pub.publish(my_dict)
            self.rate.sleep()

if __name__ == "__main__":
    try:
        vd_ = VisionDriver()
        vd_.init_ros()
        vd_.run()

    except rospy.ROSInterruptException:
        pass
