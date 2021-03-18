#!/usr/bin/python

import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, Int8
from nav_msgs.msg import Odometry
from marker_detection import*

class marker_detection_ros_interface:

    def __init__(self):

        #Init ROS node
        rospy.init_node('marker_detection')
        self.cycle_time = (1./10.)
        self.marker_detection = marker_detection()
        
        #Variables for images 
        self.bottom_img = None
        self.front_img = None
        
        #Variables for marker detection
        self.ground_truth = Odometry()
        self.aruco_pose = PoseStamped()
        self.draw_markers = True

        #Configuration of marker board 1(GPS2Vision) 2(Vision navigation) 3(Vision landing 1)
        #4(Vision landing 2) and 5(Vision landing 3)
        self.aruco_board = 1
        
        #Subscribers
        rospy.Subscriber("/mono_cam_bottom/image_raw", Image, self.bottom_img_callback)
        rospy.Subscriber("/mono_cam_front/image_raw", Image, self.front_img_callback)
        rospy.Subscriber('/onboard/aruco_board', Int8, self.aruco_board_callback)
        rospy.Subscriber('/odom', Odometry, self.ground_truth_callback)

        #Publishers
        self.aruco_marker_image_pub = rospy.Publisher('/onboard/aruco_marker_image', Image, queue_size=1)
        self.aruco_marker_pose_pub = rospy.Publisher('/onboard/aruco_marker_pose', PoseStamped, queue_size=1)
        self.aruco_marker_found_pub = rospy.Publisher('/onboard/aruco_board_found', Bool, queue_size=1)
        self.aruco_marker_pose_stable_pub = rospy.Publisher('/onboard/aruco_marker_pose_stable', Bool, queue_size=1)
        self.aruco_marker_board_center_pub = rospy.Publisher('/onboard/aruco_marker_board_center', PoseStamped, queue_size=1)

        rospy.Timer(rospy.Duration(self.cycle_time), self.timer_callback)
        rospy.spin()

    def aruco_board_callback(self, data):
        self.aruco_board = data.data

    def bottom_img_callback(self, data):
        self.bottom_img = data

    def front_img_callback(self, data):
        self.front_img = data
        
    def ground_truth_callback(self, data):
        self.ground_truth = data

    def timer_callback(self, event):

        aruco_board = self.aruco_board

        #Using front camera (GPS2Vision)
        if aruco_board == 1:
            self.marker_detection.find_aruco_markers(self.front_img, 
                                                     self.marker_detection.aruco_board_gps2vision, 
                                                     self.marker_detection.camera_matrix_front, 
                                                     self.marker_detection.distortion_coefficients_front, 
                                                     self.draw_markers,
                                                     True)
            if self.marker_detection.aruco_board_found:
                self.marker_detection.estimate_marker_pose(aruco_board, self.marker_detection.T_gps2visionMarker_to_ground, self.ground_truth)
                self.marker_detection.calculate_rolling_average(self.marker_detection.marker_pose)
        #Using bottom camera (Vision navigation)
        elif aruco_board == 2:
            self.marker_detection.find_aruco_markers(self.bottom_img, 
                                                     self.marker_detection.aruco_board_vision, 
                                                     self.marker_detection.camera_matrix_bottom, 
                                                     self.marker_detection.distortion_coefficients_bottom,
                                                     self.draw_markers)
            if self.marker_detection.aruco_board_found:
                self.marker_detection.estimate_marker_pose(aruco_board, ground_truth = self.ground_truth)
        #Using bottom camera (Vision landing 1)
        elif aruco_board == 3:
            self.marker_detection.find_aruco_markers(self.front_img, 
                                                     self.marker_detection.aruco_board_landing, 
                                                     self.marker_detection.camera_matrix_front, 
                                                     self.marker_detection.distortion_coefficients_front,
                                                     self.draw_markers)
            if self.marker_detection.aruco_board_found:
                self.marker_detection.estimate_marker_pose(aruco_board, self.T_landingMarker1_to_ground, self.ground_truth)
        #Using bottom camera (Vision landing 2)
        elif aruco_board == 4:
            self.marker_detection.find_aruco_markers(self.front_img, 
                                                     self.marker_detection.aruco_board_landing, 
                                                     self.marker_detection.camera_matrix_front, 
                                                     self.marker_detection.distortion_coefficients_front, 
                                                     self.draw_markers)
            if self.marker_detection.aruco_board_found:
                self.marker_detection.estimate_marker_pose(aruco_board, self.marker_detection.T_landingMarker2_to_ground, self.ground_truth)
        #Using bottom camera (Visiom landing 3)
        elif aruco_board == 5:
            self.marker_detection.find_aruco_markers(self.front_img, 
                                                     self.marker_detection.aruco_board_landing, 
                                                     self.marker_detection.camera_matrix_front, 
                                                     self.marker_detection.distortion_coefficients_front, 
                                                     self.draw_markers)
            if self.marker_detection.aruco_board_found:
                self.marker_detection.estimate_marker_pose(aruco_board, self.marker_detection.T_landingMarker3_to_ground, self.ground_truth)

        #Publish image from camera is wanted
        if self.draw_markers:
            self.aruco_marker_image_pub.publish(self.marker_detection.ros_img)

        #Publish only if using GPS2Vision aruco board (if the estimated pose is stable(based on rolling average))
        if aruco_board == 1:
            self.aruco_marker_pose_stable_pub.publish(self.marker_detection.aruco_marker_pose_stable)

        #Publish only if wanted aruco board is found
        if self.marker_detection.aruco_board_found:
            self.aruco_marker_pose_pub.publish(self.marker_detection.marker_pose)

        #Publish if board is found
        self.aruco_marker_found_pub.publish(self.marker_detection.aruco_board_found)

if __name__ == "__main__":
    mdri = marker_detection_ros_interface()
