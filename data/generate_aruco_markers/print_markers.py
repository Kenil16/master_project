#!/usr/bin/python

import cv2
import cv2.aruco as aruco
import numpy as np
from fpdf import FPDF 
from glob import glob 
from PIL import Image

class print_markers:

    def __init__(self):

        #To be used in the the simulation the markers size and separation must be defined in pixels.
        #Here a marker and separation size of 0.2 m and 0.02 m have been chosen respectively.
        #The scale factor in Blender for x and why are approx 0.86 and 0.43 m

        self.dictionary = cv2.aruco.Dictionary_get(cv2.aruco.DICT_5X5_1000)
        self.markers_x = 1 #25
        self.markers_y = 1 #25
        self.marker_separation_meters = 0.1 #0.01
        self.marker_length_meters = 0.2 #0.02
        self.marker_length = 755.90551181  #Approx 0.02 meters in pixels 755.90551181
        self.marker_separation = 377.95275591 #Approx 0.01 meters in pixels 377.95275591
        self.first_marker = 202
        self.aruco_board = cv2.board = aruco.GridBoard_create(self.markers_x, self.markers_y, self.marker_length, self.marker_separation, self.dictionary, firstMarker = self.first_marker)

    def print_aruco_board(self):
        
        image_width = self.markers_x*(self.marker_length + self.marker_separation) - self.marker_separation
        image_height = self.markers_y*(self.marker_length + self.marker_separation) - self.marker_separation
        
        length_x = self.markers_x*(self.marker_length_meters) + self.markers_x*(self.marker_separation_meters) - self.marker_separation_meters
        length_y = self.markers_y*(self.marker_length_meters) + self.markers_y*(self.marker_separation_meters) - self.marker_separation_meters 
        
        print("Length in x: " + str(length_x))
        print("Length in y: " + str(length_y))
        print("Pixels in y: " + str(image_height))
        print("Pixels in x: " + str(image_width))
        
        img = self.aruco_board.draw((int(image_width),int(image_height)))
        font = cv2.FONT_HERSHEY_SIMPLEX
        cv2.putText(img,'OpenCV ArUco marker (' + str((self.first_marker-200)/25) + "," + str(self.first_marker-200) + ')' ,(8,20), font, 0.35,(255,255,255),1,cv2.LINE_AA)
        
        #cv2.imwrite('../../blender_models/aruco_boards_used/grid_board_vision' + str(self.first_marker) +'.png',img) # Set back to grid_board_new_
        cv2.imwrite('grid_board_single_marker' + str(self.first_marker) +'.png',img) # Set back to grid_board_new_

        image = glob('grid_board_single_marker' + str(self.first_marker) +'.png')
        
        image = glob('grid_board_landing_2_1.png')
        
        pdf = FPDF()
        for img in image:
            pdf.add_page()
            pdf.image(img,60,13,82.5)
        pdf.output("landing_board2.pdf", "F")

    def print_multible_markers(self):
        
        image_width = self.markers_x*(self.marker_length + self.marker_separation) - self.marker_separation
        image_height = self.markers_y*(self.marker_length + self.marker_separation) - self.marker_separation
        
        length_x = self.markers_x*(self.marker_length_meters) + self.markers_x*(self.marker_separation_meters) - self.marker_separation_meters
        length_y = self.markers_y*(self.marker_length_meters) + self.markers_y*(self.marker_separation_meters) - self.marker_separation_meters 
        
        print("Length in x: " + str(length_x))
        print("Length in y: " + str(length_y))
        print("Pixels in y: " + str(image_height))
        print("Pixels in x: " + str(image_width))
        
        first_marker = 221
        for i in range(0,1):
            
            self.first_marker = first_marker + 25*i
            print(self.first_marker)
            self.aruco_board = aruco.GridBoard_create(self.markers_x, self.markers_y, self.marker_length, self.marker_separation, self.dictionary, firstMarker = self.first_marker)
            
            img = self.aruco_board.draw((int(image_width),int(image_height)))
            font = cv2.FONT_HERSHEY_SIMPLEX
            cv2.putText(img,'OpenCV ArUco marker ' + str(self.first_marker)  + ' (' + str((self.first_marker-200)/25) + "," + str((self.first_marker - 200 - (25*i))) + ') DICT_5X5_1000 ' ,(8,20), font, 0.35,(255,255,255),1,cv2.LINE_AA)
            
            #cv2.imwrite('../../blender_models/aruco_boards_used/grid_board_vision' + str(self.first_marker) +'.png',img) # Set back to grid_board_new_
            cv2.imwrite('grid_board_single_marker' + str(self.first_marker) +'.png',img) # Set back to grid_board_new_

            image = glob('grid_board_single_marker' + str(self.first_marker) +'.png')
            pdf = FPDF()
            for img in image:
                pdf.add_page()
                pdf.image(img,5,47.5,200)
            pdf.output('grid_board_single_marker' + str(self.first_marker) +'.pdf', "F")

    def print_multible_boards(self):
        
        first_marker = 200
        for _ in range(25*25):
            
            self.dictionary = cv2.aruco.Dictionary_get(cv2.aruco.DICT_5X5_1000)
            self.markers_x = 1
            self.markers_y = 1
            self.marker_separation_meters = 0.02
            self.marker_length_meters = 0.2
            self.marker_length = 377.95275591 #Approx 0.1 meters in pixels 
            self.marker_separation = 75.590551181 #Approx 0.08 meters in pixels
            self.aruco_board = cv2.board = aruco.GridBoard_create(self.markers_x, self.markers_y, self.marker_length, self.marker_separation, self.dictionary, firstMarker = first_marker)
            
            image_width = self.markers_x*(self.marker_length + self.marker_separation)
            image_height = self.markers_y*(self.marker_length + self.marker_separation)
            
            length_x = self.markers_x*(self.marker_length_meters) + self.markers_x*(self.marker_separation_meters) - self.marker_separation_meters
            length_y = self.markers_y*(self.marker_length_meters) + self.markers_y*(self.marker_separation_meters) - self.marker_separation_meters 
            
            print("Length in x: " + str(length_x))
            print("Length in y: " + str(length_y))
            print("Pixels in y: " + str(image_height))
            print("Pixels in x: " + str(image_width))
            
            img = self.aruco_board.draw((int(image_width),int(image_height)))
            cv2.imwrite('big_board/grid_board_new_' + str(first_marker) +'.png',img)
            first_marker += 1
if __name__ == "__main__":

    pm = print_markers()
    #pm.print_aruco_board()
    pm.print_multible_markers()

