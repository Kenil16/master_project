#!/usr/bin/python

import cv2
import cv2.aruco as aruco
import numpy as np

class print_markers:

    def __init__(self):

        #To be used in the the simulation the markers size and separation must be defined in pixels.
        #Here a marker and separation size of 0.2 m and 0.02 m have been chosen respectively.
        #The scale factor in Blender for x and why are approx 0.86 and 0.43 m

        self.dictionary = cv2.aruco.Dictionary_get(cv2.aruco.DICT_5X5_1000)
        self.markers_x = 4
        self.markers_y = 2
        self.marker_separation_meters = 0.08
        self.marker_length_meters = 0.2
        self.marker_length = 755.90551181 #Approx 0.2 meters in pixels 
        self.marker_separation = 302.36220472 #Approx 0.04 meters in pixels 
        self.aruco_board = cv2.board = aruco.GridBoard_create(self.markers_x, self.markers_y, self.marker_length, self.marker_separation, self.dictionary, firstMarker=500)

    def print_aruco_board(self):
        
        image_width = self.markers_x*(self.marker_length + self.marker_separation)
        image_height = self.markers_y*(self.marker_length + self.marker_separation) - 162
        
        length_x = self.markers_x*(self.marker_length_meters) + self.markers_x*(self.marker_separation_meters) - self.marker_separation_meters
        length_y = self.markers_y*(self.marker_length_meters) + self.markers_y*(self.marker_separation_meters) - self.marker_separation_meters 
        
        print("Length in x: " + str(length_x))
        print("Length in y: " + str(length_y))
        print("Pixels in y: " + str(image_height))
        print("Pixels in x: " + str(image_width))
        
        img = self.aruco_board.draw((int(image_width),int(image_height)))
        cv2.imwrite('grid_board.png',img)

if __name__ == "__main__":

    pm = print_markers()
    pm.print_aruco_board()

