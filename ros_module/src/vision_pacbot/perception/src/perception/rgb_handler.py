#! /usr/bin/env python3.8
# -*- coding: utf-8 -*-
'''
| author:
| Belal HMEDAN, 
| LIG lab/ Marvin Team, 
| France, 2022.
| RGB image processing script.
'''

import numpy as np
import cv2
from functools import cmp_to_key
from math import floor
import sys
##============================
## Class RGB Image Processor |
##============================
class RGBProcessor:

    def __init__(self):
        """
        Class RGBProcessor: Read and Process rgb image to get 2D world state.
        """
        self.greenRect = None
        self.cellList  = None
        self.cellCenters = None
        self.hand = False
        self.motion_detected = False
        self.cellsState = {
        'p_00_00':'g', 'p_00_01':'g', 'p_00_02':'g', 'p_00_03':'g', 'p_00_04':'g', 'p_00_05':'g', 'p_00_06':'g', 'p_00_07':'g', 'p_00_08':'g', 'p_00_09':'g', 'p_00_10':'g', 'p_00_11':'g', 'p_00_12':'g', 'p_00_13':'g', 'p_00_14':'g', 'p_00_15':'g', 'p_00_16':'g', 'p_00_17':'g', 'p_00_18':'g', 'p_00_19':'g', 'p_00_20':'g', 'p_00_21':'g', 'p_00_22':'g',
        'p_01_00':'g', 'p_01_01':'g', 'p_01_02':'g', 'p_01_03':'g', 'p_01_04':'g', 'p_01_05':'g', 'p_01_06':'g', 'p_01_07':'g', 'p_01_08':'g', 'p_01_09':'g', 'p_01_10':'g', 'p_01_11':'g', 'p_01_12':'g', 'p_01_13':'g', 'p_01_14':'g', 'p_01_15':'g', 'p_01_16':'g', 'p_01_17':'g', 'p_01_18':'g', 'p_01_19':'g', 'p_01_20':'g', 'p_01_21':'g', 'p_01_22':'g',
        'p_02_00':'g', 'p_02_01':'g', 'p_02_02':'g', 'p_02_03':'g', 'p_02_04':'g', 'p_02_05':'g', 'p_02_06':'g', 'p_02_07':'g', 'p_02_08':'g', 'p_02_09':'g', 'p_02_10':'g', 'p_02_11':'g', 'p_02_12':'g', 'p_02_13':'g', 'p_02_14':'g', 'p_02_15':'g', 'p_02_16':'g', 'p_02_17':'g', 'p_02_18':'g', 'p_02_19':'g', 'p_02_20':'g', 'p_02_21':'g', 'p_02_22':'g',
        'p_03_00':'g', 'p_03_01':'g', 'p_03_02':'g', 'p_03_03':'g', 'p_03_04':'g', 'p_03_05':'g', 'p_03_06':'g', 'p_03_07':'g', 'p_03_08':'g', 'p_03_09':'g', 'p_03_10':'g', 'p_03_11':'g', 'p_03_12':'g', 'p_03_13':'g', 'p_03_14':'g', 'p_03_15':'g', 'p_03_16':'g', 'p_03_17':'g', 'p_03_18':'g', 'p_03_19':'g', 'p_03_20':'g', 'p_03_21':'g', 'p_03_22':'g',
        'p_04_00':'g', 'p_04_01':'g', 'p_04_02':'g', 'p_04_03':'g', 'p_04_04':'g', 'p_04_05':'g', 'p_04_06':'g', 'p_04_07':'g', 'p_04_08':'g', 'p_04_09':'g', 'p_04_10':'g', 'p_04_11':'g', 'p_04_12':'g', 'p_04_13':'g', 'p_04_14':'g', 'p_04_15':'g', 'p_04_16':'g', 'p_04_17':'g', 'p_04_18':'g', 'p_04_19':'g', 'p_04_20':'g', 'p_04_21':'g', 'p_04_22':'g',
        'p_05_00':'g', 'p_05_01':'g', 'p_05_02':'g', 'p_05_03':'g', 'p_05_04':'g', 'p_05_05':'g', 'p_05_06':'g', 'p_05_07':'g', 'p_05_08':'g', 'p_05_09':'g', 'p_05_10':'g', 'p_05_11':'g', 'p_05_12':'g', 'p_05_13':'g', 'p_05_14':'g', 'p_05_15':'g', 'p_05_16':'g', 'p_05_17':'g', 'p_05_18':'g', 'p_05_19':'g', 'p_05_20':'g', 'p_05_21':'g', 'p_05_22':'g',
        'p_06_00':'g', 'p_06_01':'g', 'p_06_02':'g', 'p_06_03':'g', 'p_06_04':'g', 'p_06_05':'g', 'p_06_06':'g', 'p_06_07':'g', 'p_06_08':'g', 'p_06_09':'g', 'p_06_10':'g', 'p_06_11':'g', 'p_06_12':'g', 'p_06_13':'g', 'p_06_14':'g', 'p_06_15':'g', 'p_06_16':'g', 'p_06_17':'g', 'p_06_18':'g', 'p_06_19':'g', 'p_06_20':'g', 'p_06_21':'g', 'p_06_22':'g',
        'p_07_00':'g', 'p_07_01':'g', 'p_07_02':'g', 'p_07_03':'g', 'p_07_04':'g', 'p_07_05':'g', 'p_07_06':'g', 'p_07_07':'g', 'p_07_08':'g', 'p_07_09':'g', 'p_07_10':'g', 'p_07_11':'g', 'p_07_12':'g', 'p_07_13':'g', 'p_07_14':'g', 'p_07_15':'g', 'p_07_16':'g', 'p_07_17':'g', 'p_07_18':'g', 'p_07_19':'g', 'p_07_20':'g', 'p_07_21':'g', 'p_07_22':'g',
        'p_08_00':'g', 'p_08_01':'g', 'p_08_02':'g', 'p_08_03':'g', 'p_08_04':'g', 'p_08_05':'g', 'p_08_06':'g', 'p_08_07':'g', 'p_08_08':'g', 'p_08_09':'g', 'p_08_10':'g', 'p_08_11':'g', 'p_08_12':'g', 'p_08_13':'g', 'p_08_14':'g', 'p_08_15':'g', 'p_08_16':'g', 'p_08_17':'g', 'p_08_18':'g', 'p_08_19':'g', 'p_08_20':'g', 'p_08_21':'g', 'p_08_22':'g',
        'p_09_00':'g', 'p_09_01':'g', 'p_09_02':'g', 'p_09_03':'g', 'p_09_04':'g', 'p_09_05':'g', 'p_09_06':'g', 'p_09_07':'g', 'p_09_08':'g', 'p_09_09':'g', 'p_09_10':'g', 'p_09_11':'g', 'p_09_12':'g', 'p_09_13':'g', 'p_09_14':'g', 'p_09_15':'g', 'p_09_16':'g', 'p_09_17':'g', 'p_09_18':'g', 'p_09_19':'g', 'p_09_20':'g', 'p_09_21':'g', 'p_09_22':'g',
        'p_10_00':'g', 'p_10_01':'g', 'p_10_02':'g', 'p_10_03':'g', 'p_10_04':'g', 'p_10_05':'g', 'p_10_06':'g', 'p_10_07':'g', 'p_10_08':'g', 'p_10_09':'g', 'p_10_10':'g', 'p_10_11':'g', 'p_10_12':'g', 'p_10_13':'g', 'p_10_14':'g', 'p_10_15':'g', 'p_10_16':'g', 'p_10_17':'g', 'p_10_18':'g', 'p_10_19':'g', 'p_10_20':'g', 'p_10_21':'g', 'p_10_22':'g'
        }

    def HSV_mask(self, img, color):
        """
        Function: check_HSV, to get the ratio of the marked pixels of specific color in the frame,
            using HSV space.
        ---
        Parameters:
        @param: img, ndarray, image frame of shape [height, width, 3(BGR)].
        @param: color, string, defines the color from ['red', 'green', 'blue', 'yellow', 'white', 'light', 'olive', 'gray']
        ---
        @return: mask, ndarray, the masked image for the given color.
        """
        colors = ['red', 'green', 'blue', 'yellow', 'white', 'light', 'olive', 'gray']
        if not color in colors:
            print('Please Note that the color has to be either: [red, green, blue, yellow, light, olive or white]\n')
            return np.zeros_like(img)[...,0]
        # Dictionary to map the range of the Hue, Saturation, Value(illumination) of each color.
        colorDictionary = {
                        "gray"  : ( [  0,   0,   0], [180, 110, 255] ),
                        "red"   : ( [165,   0,   0], [180, 255, 255] ),
                        "green" : ( [ 65, 160,   0], [ 86, 255, 255] ),
                        "blue"  : ( [ 80, 160,   0], [115, 255, 255] ),
                        "yellow": ( [ 15, 140,   0], [ 35, 255, 255] ),
                        "light" : ( [ 46, 101, 170], [ 54, 255, 255] ), # "light_green"
                        "olive" : ( [ 36, 120,   0], [ 46, 255, 255] ), # "olive_green"
                        "white" : ( [  0,   0,  70], [180, 100, 255] )
        }

        hsv_img = cv2.cvtColor(img,cv2.COLOR_BGR2HSV)

        # Get current lower and upper range values:
        LowRange = np.array(colorDictionary[color][0])
        UppRange = np.array(colorDictionary[color][1])

        # checking the marked pixels in the captured_hsv_frame
        if color != "red" :
            mask = cv2.inRange(hsv_img, LowRange, UppRange)
        else:
            # Red color has two ranges
            mask1 = cv2.inRange(hsv_img, LowRange, UppRange)
            LowRange[0] = 0
            UppRange[0] = 10
            mask2 = cv2.inRange(hsv_img, LowRange, UppRange)
            mask  = cv2.bitwise_or(mask1, mask2)
        return mask

    def Morphology(self, mask, k=15, iters=1):
        """
        Function: Morphology, to do morphological closing to the mask.
        ---
        Parameters:
        @param: mask, nd array binary mask resulting from HSV masking.
        @param: k, integer, kernel (structuring element) size.
        @param: iters, integer, operation iterations.
        
        ---
        @return: mask, nd array binary mask resulting from HSV masking.
        """
        # Get the structuring element:
        openKernel  = cv2.getStructuringElement(cv2.MORPH_RECT, ksize=(k, k))
        closeKernel = cv2.getStructuringElement(cv2.MORPH_RECT, ksize=(3*k, 3*k))
        # Perform closing:
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, openKernel, None, None, iters, cv2.BORDER_REFLECT101)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, closeKernel, None, None, iters, cv2.BORDER_REFLECT101)

        return mask

    def rect_sort(self, a, b):
        """
        Function: findMarkersContour, to find the contours of the red markers.
        Reference: https://stackoverflow.com/a/67105599
        ---
        Parameters:
        @param: mask, nd array binary mask resulting from HSV masking, and Morpholigical closing.
        @param: minArea, integer, the min size of the rectangle to be considered.
        
        ---
        @return: boundRectsSorted, list, bounding rectangles list sorted.
        """
        if abs(a[1] - b[1]) <= 15:
            return a[0] - b[0]
        return a[1] - b[1]

    def findMarkersContour(self, mask, minArea=100):
        """
        Function: findMarkersContour, to find the contours of the red markers.
        ---
        Parameters:
        @param: mask, nd array binary mask resulting from HSV masking, and Morpholigical closing.
        @param: minArea, integer, the min size of the rectangle to be considered.
        
        ---
        @return: None # boundRectsSorted, list, bounding rectangles list sorted.
        """
        # Find the big contours/blobs on the filtered image:
        contours = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[0]

        # Bounding Rects are stored here:
        boundRectsList = []

        # Process each contour 1-1:
        for c in contours:

            # Approximate the contour to a polygon:
            contoursPoly = cv2.approxPolyDP(c, 3, True)

            # Convert the polygon to a bounding rectangle:
            boundRect = cv2.boundingRect(contoursPoly)

            # Get the bounding rect's data:
            rectWidth  = boundRect[2]
            rectHeight = boundRect[3]

            # Estimate the bounding rect area:
            rectArea = rectWidth * rectHeight

            # Filter blobs by area:
            if rectArea > minArea:
                #Store the rect:
                boundRectsList.append(boundRect)
        # Sort the list based on ascending y values:
        boundRectsSorted = sorted(boundRectsList, key=cmp_to_key(self.rect_sort))
        self.greenRect = boundRectsSorted

    def cropPlatform(self, img, output_size=(759, 384)): # 32 x 24, 32 x 12 (759, 384)
        """
        Function: cropPlatform, to crop the green platform.
        ---
        Parameters:
        @param: img, nd array to be cropped.
        @param: boundRectsSorted, list of bounding box coordinates sorted.
        @param: output_size, tuple, the output ROI size.
        
        ---
        @return: cropped, nd array, croppedimage.
        """
        areas = [(self.greenRect[i][2]) * (self.greenRect[i][3]) for i in range(len(self.greenRect))]
        idx = areas.index(max(areas))
        myRect = self.greenRect[idx]
        
        x0 = myRect[0]
        y0 = myRect[1]
        w  = myRect[2]
        x1 = x0 + w
        h  = myRect[3]
        y1 = y0 + h

        cropped = img[y0:y1, x0:x1]

        cropped = cv2.resize(cropped, output_size)

        return cropped

    def bgr_to_hsv(self, bgr):
        """
        Function: bgr_to_hsv, to obtain the HSV value of one pixel from BGR.
        ---
        Parameters:
        @param: bgr,list of the three colors value, blue, green, red.
        
        ---
        @return: hsv,list of the value, Hue, Saturation, Value.
        """
        b, g, r = bgr
        r, g, b = r/255.0, g/255.0, b/255.0
        mx = max(r, g, b)
        mn = min(r, g, b)
        df = mx-mn
        if mx == mn:
            h = 0
        elif mx == r:
            h = (60 * ((g-b)/df) + 360) % 360
        elif mx == g:
            h = (60 * ((b-r)/df) + 120) % 360
        elif mx == b:
            h = (60 * ((r-g)/df) + 240) % 360
        if mx == 0:
            s = 0
        else:
            s = (df/mx)*255 # (df/mx)*100
        v = mx*255 # mx*100
        h = h / 2
        return [floor(h), floor(s), floor(v)]

    def get_dominant_color(self, inp_img):
        """
        Function: get_dominant_color, to get the dominant color pixel values.
        https://stackoverflow.com/a/61730849
        ---
        Parameters:
        @param: inp_img, nd-array, the input image to quantify its color.
        
        ---
        @return: dominant_color, list, hsv values of the dominant color.
        """
        img = inp_img.copy()
        out = cv2.resize(img, (1, 1), 0)
        dominant_color = self.bgr_to_hsv(out[0, 0])
        return dominant_color

    def cmp_ranges(self, inp, ranges):
        """
        Function: cmp_ranges, to compare the HSV ranges.
        ---
        Parameters:
        @param: inp, list, the input hsv list values to be compared.
        @param: ranges, tuple of two lists, the ranges to compare the input.
        
        ---
        @return: output, Bool, True if the input hsv values within the ranges.
        """
        low  = ranges[0]
        high = ranges[1]
        output = True
        for idx, elem in enumerate(inp):
            if (elem < low[idx] or elem > high[idx] ):
                output = False
        return output

    def gridWorkspace(self, roi, gridSize=(23, 11), shift=[10, 10], verbose=False):
        """
        Function: gridWorkspace, to find the contours of the red markers.
        ---
        Parameters:
        @param: roi, nd array, cropped region of interest.
        @param: gridSize, tuple, lenght/width or the Workspace.
        @param: shift, to make static error compensation for alignment.
        @param: verbose, to visualize the grid.
        ---
        @return: None # cellList, list, cells coordinates list,
                cellCenters, list, cells centers list.
        """
        # # Store a deep copy for results:
        roi_copy = roi.copy()

        # Divide the image into a grid:
        verticalCells   = gridSize[1]
        horizontalCells = gridSize[0]

        ## Cell dimensions
        bigRectWidth  =  roi_copy.shape[1]
        bigRectHeight =  roi_copy.shape[0]

        cellWidth  = (bigRectWidth // horizontalCells) - 1
        cellHeight =  (bigRectHeight // verticalCells)  - 1

        # Store the cells here:
        cellList = []

        # Store cell centers here:
        cellCenters = []

        x_shift, y_shift = shift

        ### Text Parameters
        # font
        font = cv2.FONT_HERSHEY_SIMPLEX
        # fontScale
        fontScale = 0.4
        # Blue color in BGR
        color = (0, 0, 0)
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

                # Crop current cell:
                currentCell = roi[cY + cellHeight//4 :cY + 3*cellHeight//4, cX + cellWidth//4:cX + 3*cellWidth//4]

                # into the cell list:
                cellList.append(currentCell)

                # Store cell center:
                cellCenters.append((cX + cellWidth//2, cY + cellHeight//2))
                if(verbose):
                    # Draw RGB Cells
                    text = str(self.get_dominant_color(currentCell)[0])
                    cv2.rectangle(roi_copy, (cX, cY), (cX + cellWidth, cY + cellHeight), color, 1)
                    roi_copy = cv2.putText(roi_copy, text, (cX+x_shift-5, cY+y_shift+5), font,
                        fontScale, color, thickness, cv2.LINE_4)
        if(verbose):
            cv2.namedWindow("grid", cv2.WINDOW_NORMAL)
            cv2.imshow("grid", roi_copy)
            key = cv2.waitKey(1)
            # Press esc or 'q' to close the image window
            if key & 0xFF == ord('q') or key == 27:
                cv2.destroyAllWindows()
                sys.exit()

        self.cellList = cellList
        if(self.cellCenters is None):
            self.cellCenters = cellCenters

    def cellAnalyser(self):
        """
        Function: cellAnalyser, to recognize the color of each cell.
        ---
        Parameters:
        @param: None
        
        ---
        @return: None
        """
        # TODO: take this dictionary fom external Json
        # HSV dictionary - color ranges and color name:
        colorDictionary = {
                        "red"   : ( [165,   0,   0], [180, 255, 255] ),
                        "green" : ( [ 65, 160,   0], [ 86, 255, 255] ),
                        "blue"  : ( [ 80, 160,   0], [115, 255, 255] ),
                        "yellow": ( [ 15, 140,   0], [ 35, 255, 255] ),
                        "light" : ( [ 46, 101, 170], [ 54, 255, 255] ), # "light_green"
                        "olive" : ( [ 36, 120,   0], [ 46, 255, 255] ), # "olive_green"
                        "white" : ( [  0,   0,  70], [180, 100, 255] )
                        }
        # Cell counter:
        cellCounter = 0

        for c in range(len(self.cellList)):

            # Get current Cell:
            currentCell = self.cellList[c]

            dom = self.get_dominant_color(currentCell)
            for color in colorDictionary:
                if(dom[0]<10 and dom[1]>80):# 70
                    pos = list(self.cellsState)[c]
                    self.cellsState[pos] = "r"
                elif self.cmp_ranges(dom, colorDictionary[color]):
                    pos = list(self.cellsState)[c]
                    self.cellsState[pos] = color[0]
                
            # Increase cellCounter
            cellCounter += 1

    def robotArmDetector(self, roi):
        """
        Function: robotArmDetector, to detect  if robot arm is present in the workspace.
        ---
        Parameters:
        @param: roi, nd array, cropped region of interest.

        ---
        @return: None
        """
        mask = self.HSV_mask(roi, "gray")
        filtered = cv2.medianBlur(mask, 11)
        white_pixles = np.count_nonzero(filtered)
        detect_thresh = 12000
        
        if(white_pixles > detect_thresh):
            print("white pixles= {}".format(white_pixles))
            self.hand = True

    def handDetector(self, roi):
        """
        Function: handDetector, to detect whether if human hand is present in the workspace.
        ---
        Parameters:
        @param: roi, nd array, cropped region of interest.

        ---
        @return: boolean, True if hand is detected, and False otherwise.
        """
        
        #converting from gbr to hsv color space
        img_HSV = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
        #skin color range for hsv color space  (0, 15, 0), (17,170,255)
        HSV_mask_1 = cv2.inRange(img_HSV,(0, 0, 0), (14,140,255))
        HSV_mask_2 = cv2.inRange(img_HSV,(150, 0, 0), (180,140,255))
        HSV_mask = cv2.bitwise_or(HSV_mask_1, HSV_mask_2)
        HSV_mask = cv2.morphologyEx(HSV_mask, cv2.MORPH_OPEN, np.ones((3,3), np.uint8))

        #converting from gbr to YCbCr color space
        img_YCrCb = cv2.cvtColor(roi, cv2.COLOR_BGR2YCrCb)
        #skin color range for hsv color space (0, 135, 85), (255,180,135)
        YCrCb_mask = cv2.inRange(img_YCrCb, (0, 140, 105), (255,155,125))
        YCrCb_mask = cv2.morphologyEx(YCrCb_mask, cv2.MORPH_OPEN, np.ones((3,3), np.uint8))

        #merge skin detection (YCbCr and hsv)
        global_mask = cv2.bitwise_and(YCrCb_mask,HSV_mask)
        global_mask = cv2.medianBlur(global_mask,7)
        global_mask = cv2.GaussianBlur(global_mask, (5, 5), 0)
        global_mask[global_mask>0]=255
        global_mask = cv2.morphologyEx(global_mask, cv2.MORPH_OPEN, np.ones((11, 11), np.uint8))

        # Find the big contours/blobs on the filtered image:
        contours = cv2.findContours(global_mask, cv2.RETR_CCOMP, cv2.CHAIN_APPROX_SIMPLE)[0]
        
        minArea = 1000

        # Bounding Rects are stored here:
        boundRectsList = []

        # Process each contour 1-1:
        for c in contours:

            # Approximate the contour to a polygon:
            contoursPoly = cv2.approxPolyDP(c, 3, True)

            # Convert the polygon to a bounding rectangle:
            boundRect = cv2.boundingRect(contoursPoly)

            # Get the bounding rect's data:
            rectWidth = boundRect[2]
            rectHeight = boundRect[3]

            # Estimate the bounding rect area:
            rectArea = rectWidth * rectHeight

            # Filter blobs by area:
            if rectArea > minArea:
                #Store the rect:
                boundRectsList.append(boundRect)
            # TODO: Get and store the cells that are covered by the hand.

        if(len(boundRectsList)!=0):
            self.hand = True
            return self.hand

        self.hand = False
        return self.hand

    def get_rgb_info(self, rgb):
        """
        Function: get_rgb_info, to call other functions to do the segmentation.
        ---
        Parameters:
        @param: rgb, nd array, rgb input image.
        
        ---
        @return: None.
        """
        if(self.greenRect is None):
            mask = self.HSV_mask(rgb, "green")
            mask = self.Morphology(mask)
            self.findMarkersContour(mask)

        roi = self.cropPlatform(rgb)

        # Detect Hand
        self.hand = self.handDetector(roi)
        self.robotArmDetector(roi)

        # No Hand Detected
        if(not self.hand and not self.motion_detected):
            self.gridWorkspace(roi)
            if(self.cellList is None):
                print("\nEmpty Cells List!!\n")
                return
            else:
                self.cellAnalyser()
