#! /usr/bin/env python3.8
# -*- coding: utf-8 -*-
'''
| author:
| Belal HMEDAN, 
| LIG lab/ Marvin Team, 
| France, 2022.
| Depth image processing script.
'''

import numpy as np
import cv2
from math import floor
import sys
##==============================
## Class Depth Image Processor |
##==============================
class DepthProcessor:

    def __init__(self):
        """
        Class DepthProcessor: Read and Process depth image to get 3D world state.
        """
        self.depthRect = None
        self.cellList  = None
        self.cellCenters = None
        self.hand = False
        self.motion_detected = False
        self.corr_array = None
        self.depthMap = np.zeros((11, 23), dtype=np.uint8)
        self.cellsState = {
            'p_00_00':0, 'p_00_01':0, 'p_00_02':0, 'p_00_03':0, 'p_00_04':0, 'p_00_05':0, 'p_00_06':0, 'p_00_07':0, 'p_00_08':0, 'p_00_09':0, 'p_00_10':0, 'p_00_11':0, 'p_00_12':0, 'p_00_13':0, 'p_00_14':0, 'p_00_15':0, 'p_00_16':0, 'p_00_17':0, 'p_00_18':0, 'p_00_19':0, 'p_00_20':0, 'p_00_21':0, 'p_00_22':0,
            'p_01_00':0, 'p_01_01':0, 'p_01_02':0, 'p_01_03':0, 'p_01_04':0, 'p_01_05':0, 'p_01_06':0, 'p_01_07':1, 'p_01_08':0, 'p_01_09':0, 'p_01_10':0, 'p_01_11':0, 'p_01_12':0, 'p_01_13':0, 'p_01_14':0, 'p_01_15':1, 'p_01_16':0, 'p_01_17':0, 'p_01_18':0, 'p_01_19':0, 'p_01_20':0, 'p_01_21':0, 'p_01_22':0,
            'p_02_00':0, 'p_02_01':0, 'p_02_02':0, 'p_02_03':0, 'p_02_04':0, 'p_02_05':0, 'p_02_06':0, 'p_02_07':0, 'p_02_08':0, 'p_02_09':0, 'p_02_10':0, 'p_02_11':0, 'p_02_12':0, 'p_02_13':0, 'p_02_14':0, 'p_02_15':0, 'p_02_16':0, 'p_02_17':0, 'p_02_18':0, 'p_02_19':0, 'p_02_20':0, 'p_02_21':0, 'p_02_22':0,
            'p_03_00':0, 'p_03_01':0, 'p_03_02':0, 'p_03_03':0, 'p_03_04':0, 'p_03_05':0, 'p_03_06':0, 'p_03_07':0, 'p_03_08':0, 'p_03_09':0, 'p_03_10':0, 'p_03_11':0, 'p_03_12':0, 'p_03_13':0, 'p_03_14':0, 'p_03_15':0, 'p_03_16':0, 'p_03_17':0, 'p_03_18':0, 'p_03_19':0, 'p_03_20':0, 'p_03_21':0, 'p_03_22':0,
            'p_04_00':0, 'p_04_01':0, 'p_04_02':0, 'p_04_03':0, 'p_04_04':0, 'p_04_05':0, 'p_04_06':0, 'p_04_07':0, 'p_04_08':0, 'p_04_09':0, 'p_04_10':0, 'p_04_11':0, 'p_04_12':0, 'p_04_13':0, 'p_04_14':0, 'p_04_15':0, 'p_04_16':0, 'p_04_17':0, 'p_04_18':0, 'p_04_19':0, 'p_04_20':0, 'p_04_21':0, 'p_04_22':0,
            'p_05_00':0, 'p_05_01':0, 'p_05_02':0, 'p_05_03':0, 'p_05_04':0, 'p_05_05':0, 'p_05_06':0, 'p_05_07':0, 'p_05_08':0, 'p_05_09':0, 'p_05_10':0, 'p_05_11':0, 'p_05_12':0, 'p_05_13':0, 'p_05_14':0, 'p_05_15':0, 'p_05_16':0, 'p_05_17':0, 'p_05_18':0, 'p_05_19':0, 'p_05_20':0, 'p_05_21':0, 'p_05_22':0,
            'p_06_00':0, 'p_06_01':0, 'p_06_02':0, 'p_06_03':0, 'p_06_04':0, 'p_06_05':0, 'p_06_06':0, 'p_06_07':1, 'p_06_08':0, 'p_06_09':0, 'p_06_10':0, 'p_06_11':0, 'p_06_12':0, 'p_06_13':0, 'p_06_14':0, 'p_06_15':1, 'p_06_16':0, 'p_06_17':0, 'p_06_18':0, 'p_06_19':0, 'p_06_20':0, 'p_06_21':0, 'p_06_22':0,
            'p_07_00':0, 'p_07_01':0, 'p_07_02':0, 'p_07_03':0, 'p_07_04':0, 'p_07_05':0, 'p_07_06':0, 'p_07_07':0, 'p_07_08':0, 'p_07_09':0, 'p_07_10':0, 'p_07_11':0, 'p_07_12':0, 'p_07_13':0, 'p_07_14':0, 'p_07_15':0, 'p_07_16':0, 'p_07_17':0, 'p_07_18':0, 'p_07_19':0, 'p_07_20':0, 'p_07_21':0, 'p_07_22':0,
            'p_08_00':0, 'p_08_01':0, 'p_08_02':0, 'p_08_03':0, 'p_08_04':0, 'p_08_05':0, 'p_08_06':0, 'p_08_07':0, 'p_08_08':0, 'p_08_09':0, 'p_08_10':0, 'p_08_11':0, 'p_08_12':0, 'p_08_13':0, 'p_08_14':0, 'p_08_15':0, 'p_08_16':0, 'p_08_17':0, 'p_08_18':0, 'p_08_19':0, 'p_08_20':0, 'p_08_21':0, 'p_08_22':0,
            'p_09_00':0, 'p_09_01':0, 'p_09_02':0, 'p_09_03':0, 'p_09_04':0, 'p_09_05':0, 'p_09_06':0, 'p_09_07':0, 'p_09_08':0, 'p_09_09':0, 'p_09_10':0, 'p_09_11':0, 'p_09_12':0, 'p_09_13':0, 'p_09_14':0, 'p_09_15':0, 'p_09_16':0, 'p_09_17':0, 'p_09_18':0, 'p_09_19':0, 'p_09_20':0, 'p_09_21':0, 'p_09_22':0,
            'p_10_00':0, 'p_10_01':0, 'p_10_02':0, 'p_10_03':0, 'p_10_04':0, 'p_10_05':0, 'p_10_06':0, 'p_10_07':0, 'p_10_08':0, 'p_10_09':0, 'p_10_10':0, 'p_10_11':0, 'p_10_12':0, 'p_10_13':0, 'p_10_14':0, 'p_10_15':0, 'p_10_16':0, 'p_10_17':0, 'p_10_18':0, 'p_10_19':0, 'p_10_20':0, 'p_10_21':0, 'p_10_22':0
        }
    
    def cropPlatform(self, depth, output_size=(779, 404)): # 32 x 24, 32 x 12 (759, 384)
        """
        Function: cropPlatform, to crop the green platform.
        ---
        Parameters:
        @param: depth, nd array to be cropped.
        @param: output_size, tuple, the output ROI size.
        
        ---
        @return: boundRectsSorted, list, bounding rectangles list sorted.
        """
        areas = [(self.depthRect[i][2]) * (self.depthRect[i][3]) for i in range(len(self.depthRect))]
        idx = areas.index(max(areas))
        myRect = self.depthRect[idx]
        
        x0 = myRect[0]
        y0 = myRect[1]
        w  = myRect[2]
        x1 = x0 + w
        h  = myRect[3]
        y1 = y0 + h

        cropped = depth[y0:y1, x0:x1]
        
        cropped = self.normalize_depth(cropped)
        cropped = cv2.resize(cropped, output_size)
        return cropped

    def normalize_depth(self, depth, mini=4400, maxi=4900):
        """
        Function: normalize_depth, to normalize depth values.
        ---
        Parameters:
        @param: depth, nd array to be cropped.
        @param: mini, int, min depth value.
        @param: maxi, int, max depth value.
        
        ---
        @return: depth, list,  nd array, normalized depth.
        """
        depth[depth<mini] = mini
        depth[depth>maxi] = mini
        depth = depth- mini
        depth = depth // 2
        depth = depth.astype(np.uint8)
        return depth

    def mode(self, arr):
        """
        Function: mode, to find the mode of an array.
        ---
        Parameters:
        @param: arr, nd array, any.
        
        ---
        @return: the mode value (whatever int/float/etc) of this array.
        """
        vals,counts = np.unique(arr, return_counts=True)
        if 0 in vals:
            z_idx = np.where(vals == 0)
            vals   = np.delete(vals,   z_idx)
            counts = np.delete(counts, z_idx)
        if(not 0 in counts.shape):
            index = np.argmax(counts)
            return vals[index]
        else:
            return 0

    def corr_func(self, arr):
        """
        Function: corr_func, linear correction of discrete depth values.
        ---
        Parameters:
        @param: arr, ndarray, depth image to be corrected.
        
        ---
        @return: arr, ndarray, corrected depth image.

        """

        """
        Emperical Values
        """
        self.y_correc = - 0.25
        self.x_correc = 1.6
        
        if(self.corr_array is None):
            self.corr_array = np.zeros((arr.shape), dtype=np.int16)
            r, c = arr.shape

            for i in range(r):
                for j in range(c):
                    self.corr_array[i, j] = floor(i*self.x_correc + j*self.y_correc)
                        
        arr = arr.astype(np.int16)
        arr += self.corr_array
        arr[arr<0]=0
        arr = arr//2
        arr = arr.astype(np.uint8)
        return arr

    def mapVal(self, val, table = [ 35, 62, 80, 100, 115]):
        """
        Function: mapVal, to quantify depth values manually.
        ---
        Parameters:
        @param: val, ndarray, any.
        @param: table, list, mapping table.

        ---
        @return: None.
        """
        # table = np.array(table)
        # table = np.argsort(table) # our table is already sorted!
        # val = np.searchsorted(table, val)
        val[val <= table[0]]  = 0
        val[(val >  table[0]) & (val <=  table[1])]  = 1
        val[(val >  table[1]) & (val <=  table[2])]  = 2
        val[(val >  table[2]) & (val <=  table[3])]  = 3
        val[(val >  table[3]) & (val <=  table[4])]  = 4
        val[val  >  table[4]]  = 5

    def key2pos(self, s):
        """
        Function: key2pos, to obtain x, y cell coordinates from string of shape 'p_xx_yy'.
        ---
        Parameters:
        @param: s, string of shape 'p_xx_yy'.
        ---
        @return: tuple, (x, y), the cell coordinates.
        """
        o = s.split('_')
        return [int(o[1]), int(o[2])]

    def gridDepthWorkspace(self, roi,gridSize=(23, 11), shift=[10, 5], verbose=False):
        """
        Function: gridWorkspace, to find the contours of the red markers.
        ---
        Parameters:
        @param: roi, nd array, cropped region of interest.
        @param: gridSize, tuple, lenght/width or the Workspace.
        @param: shift, to make static error compensation for alignment.
        @param: verbose, to visualize the grid.
        ---
        @return: None.
        """
        # Store a deep copy for results:
        roi_copy = roi.copy()

        # Divide the image into a grid:
        verticalCells   = gridSize[1]
        horizontalCells = gridSize[0]

        # Cell dimensions
        bigRectWidth  =  roi_copy.shape[1]
        bigRectHeight =  roi_copy.shape[0]

        cellWidth = bigRectWidth // horizontalCells
        cellHeight = bigRectHeight // verticalCells

        # Store the cells here:
        cellList = []

        # Store cell centers here:
        cellCenters = []

        x_shift, y_shift = shift

        ### Text Parameters
        # font
        font = cv2.FONT_HERSHEY_SCRIPT_SIMPLEX
        # fontScale
        fontScale = 0.4
        # Blue color in BGR
        color = 255 # (100, 100, 255)
        # Line thickness of 2 px
        thickness = 1
        
        # Loop thru vertical dimension:
        for j in range(verticalCells):

            # Cell starting y position:
            yo = j * cellHeight + y_shift

            # Loop thru horizontal dimension:
            for i in range(horizontalCells):

                # Cell starting x position:
                xo = i * cellWidth + x_shift

                # Cell Dimensions:
                cX = int(xo)
                cY = int(yo)

                currentCell = roi[cY:cY + cellHeight, cX:cX + cellWidth]
                # into the cell list:
                cellList.append(currentCell)

                # Store cell center:
                cellCenters.append((cX + cellWidth//2, cY + cellHeight//2))

                self.depthMap[j, i] = self.mode(roi[cY + cellHeight//4 :cY + 3*cellHeight//4, cX + cellWidth//4:cX + 3*cellWidth//4])

                if(verbose):
                    # Draw RGB Cells
                    corrected = self.corr_func(255 - self.depthMap)
                    # self.mapVal(corrected)
                    text = str(corrected[j, i])
                    roi_copy = cv2.putText(roi_copy, text, (cX+5, cY+15),font, 
                        fontScale, color, thickness, cv2.LINE_AA)
                    cv2.rectangle(roi_copy, (cX, cY), (cX + cellWidth, cY + cellHeight), color, 1)
                
        if(verbose):
            cv2.namedWindow("depth_grid", cv2.WINDOW_NORMAL)
            cv2.imshow("depth_grid", roi_copy)
            key = cv2.waitKey(1)
            # Press esc or 'q' to close the image window
            if key & 0xFF == ord('q') or key == 27:
                cv2.destroyAllWindows()
                sys.exit()

        self.cellList = cellList
        if(self.cellCenters is None):
            self.cellCenters = cellCenters

        self.depthMap = self.corr_func( 255 - self.depthMap )
        self.mapVal(self.depthMap)
        for posStr in self.cellsState:
            px, py = self.key2pos(posStr)
            if((px>0 and px<8) and (py>6 and py<16)):
                self.cellsState[posStr] = str(self.depthMap[px, py])
            elif(self.depthMap[px, py]<2):
                self.cellsState[posStr] = str(self.depthMap[px, py])

    def get_depth_info(self, depth, depthRect=None):
        """
        Function: get_rgb_info, to call other functions to do the segmentation.
        ---
        Parameters:
        @param: depth, nd array uint16, depth image.
        
        ---
        @return: None.
        """
        if(self.depthRect is None):
            self.depthRect = depthRect

        depth_roi = self.cropPlatform(depth)

        # No Hand Detected
        if(not self.hand and not self.motion_detected):
            self.gridDepthWorkspace(depth_roi)
