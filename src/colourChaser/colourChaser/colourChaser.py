#!/usr/bin/env python

#twist mux is used to help with ros topics for requires downloading to root on every launch 
# download guide below 
# sudo apt update
# sudo apt install ros-humble-twist-mux
# config and launch folders were copied into this workspace from twistmux package
#https://index.ros.org/r/twist_mux/

# This code was adapted from the github page 
#https://github.com/LCAS/teaching/tree/lcas_humble/cmp3103m_ros2_code_fragments

# An example of TurtleBot 3 subscribe to camera topic, mask colours, find and display contours, and move robot to center the object in image frame
# Written for humble
# cv2 image types - http://wiki.ros.org/cv_bridge/Tutorials/ConvertingBetweenROSImagesAndOpenCVImagesPython

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge # Package to convert between ROS and OpenCV Images
import cv2
import numpy as np


class ColourChaser(Node):
    def __init__(self):
        super().__init__('colourChaser')
        
        self.turnVel = 0.0
        self.searching = False

        #Creating a dictionary to store where the colours are located or not 
        #Will be set to True if the colour is found, False if not
        self.colourSearch = {"Yellow": False, "Blue": False, "Green": False, "Red": False}

        #Creating a  another dictionary to store information about where the robot is close to colour for it to be found
        self.colourClose = {"Yellow": False, "Blue": False, "Green": False, "Red": False}

        #Setting up the publisher and subscriber
        
        # publish cmd_vel topic to move the robot
        self.pub_vel = self.create_publisher(Twist, 'colour_vel', 10)

        # create timer to publish cmd_vel topic
        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

        # subscribe to the camera topic
        self.create_subscription(Image, '/camera/image_raw', self.cameraCallback, 10)

        # subscribe to the laser topic 
        self.laser_sub = self.create_subscription(LaserScan,"/scan",self.colourSearchCallback, 1)

        # Used to convert between ROS and OpenCV images
        self.br = CvBridge()


    # a callback function for the colour searching 
    def colourSearchCallback(self, msg):
        #creating a variable to store the distance between the robot and the coloured object
        colourDistance = msg.ranges[0]

        for i in self.colourSearch:
            #if the colour is found within a certain distance, set the variable to True
            if self.colourClose[i] == True and colourDistance < 1.0:
                self.colourSearch[i] = True
                print(f"The colour {i} has been found")
                print(self.colourSearch)
            

    def cameraCallback(self, data):

        for i in self.colourClose:
            self.colourClose[i] = False

        #self.get_logger().info("cameraCallback")

        cv2.namedWindow("Image window", 1)

        # Convert ROS Image message to OpenCV image
        current_frame = self.br.imgmsg_to_cv2(data, desired_encoding='bgr8')

        # Convert image to HSV
        current_frame_hsv = cv2.cvtColor(current_frame, cv2.COLOR_BGR2HSV)
        # Create mask for range of colours (HSV low values, HSV high values)
        #current_frame_mask = cv2.inRange(current_frame_hsv,(70, 0, 50), (150, 255, 255))
        current_frame_maskYellow = cv2.inRange(current_frame_hsv,(22, 100, 100), (30, 255, 255)) # Yellow
        current_frame_maskGreen = cv2.inRange(current_frame_hsv,(34,100,0), (86,255,255)) # Green
        current_frame_maskBlue = cv2.inRange(current_frame_hsv,(100,100,0), (130,255,255)) # Blue
        current_frame_maskRed = cv2.inRange(current_frame_hsv,(170,100,0), (180,255,255)) # Red
        current_frame_maskRed2 = cv2.inRange(current_frame_hsv,(0,100,0), (5,255,255)) # Red


        contoursYellow, hierarchy = cv2.findContours(current_frame_maskYellow, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        contoursGreen, hierarchy = cv2.findContours(current_frame_maskGreen, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        contoursBlue, hierarchy = cv2.findContours(current_frame_maskBlue, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        contoursRed, hierarchy = cv2.findContours(current_frame_maskRed, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        contoursRed2, hierarchy = cv2.findContours(current_frame_maskRed2, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)


        # create a dictionary to match the contours to the colours
        colourMatch = {"Yellow": [contoursYellow], "Blue": [contoursBlue], "Green": [contoursGreen], "Red": [contoursRed, contoursRed2]}


        # Initialize an empty list to store the contours
        contours = []

        # Loop through each colour in the colourSearch dictionary
        for i in self.colourSearch:
            
            # Check if the colour has not been seen yet
            if self.colourSearch[i] == False:
                
                # Loop through each matching colour in the colourMatch dictionary
                for j in colourMatch[i]:
                    
                    # Add the contours for the matching colour to the list of contours
                    contours += j

        # Concatenate the contours for each colour into a single list
        contoursYellow + contoursGreen + contoursBlue + contoursRed + contoursRed2
        
        # Sort by area (keep only the biggest one)
        contours = sorted(contours, key=cv2.contourArea, reverse=True)[:1]

        contoursYellow = sorted(contoursYellow, key=cv2.contourArea, reverse=True)[:1]

        # Draw contour(s) (image to draw on, contours, contour number -1 to draw all contours, colour, thickness):
        current_frame_contours = cv2.drawContours(current_frame, contours, 0, (0, 255, 0), 20)
        
        if len(contours) > 0:
            if (len(contours[0]) > 10): # if largest contour is larger than 200  
                # find the centre of the contour: https://docs.opencv.org/3.4/d8/d23/classcv_1_1Moments.html
                M = cv2.moments(contours[0]) # only select the largest contour

                if M['m00'] > 0:
                    # find the centroid of the contour
                    cx = int(M['m10']/M['m00'])
                    cy = int(M['m01']/M['m00'])
                    #print("Centroid of the biggest area: ({}, {})".format(cx, cy))

                    # Draw a circle centered at centroid coordinates
                    # cv2.circle(image, center_coordinates, radius, colour, thickness) -1 px will fill the circle
                    cv2.circle(current_frame, (round(cx), round(cy)), 50, (0, 255, 0), -1)
                                
                    # find height/width of robot camera image from ros2 topic echo /camera/image_raw height: 1080 width: 1920
                    
                    self.searching = True

                    # if center of object is to the left of image center move left
                    if cx < 900:
                        self.turnVel = 0.3
                    # else if center of object is to the right of image center move right
                    elif cx >= 1200:
                        self.turnVel = -0.3
                    else: # center of object is in a 100 px range in the center of the image so dont turn
                        #print("object in the center of image")
                        self.turnVel = 0.0

                        # Call the 'colourSeen' method for each colour and contour
                        # This method checks if the contour is valid and whether it matches the given x and y coordinates
                        # If the contour matches, the method sets a flag in the 'colourClose' dictionary to indicate that the colour has been seen
                        self.colourSeen("Yellow", contoursYellow, cx, cy)

                        self.colourSeen("Blue", contoursBlue, cx, cy)

                        self.colourSeen("Green", contoursGreen, cx, cy)

                        self.colourSeen("Red", contoursRed, cx, cy)
                        
                        self.colourSeen("Red", contoursRed2, cx, cy)
                                
            else:
                #print("No Centroid Large Enough Found")
                # turn until we can see a coloured object
                self.turnVel = 0.0
                self.searching = False
                        
        else:
            #print("No Centroid Found")
            # turn until we can see a coloured object
            self.turnVel = 0.0
            self.searching = False

        # show the cv images
        current_frame_contours_small = cv2.resize(current_frame_contours, (0,0), fx=0.4, fy=0.4) # reduce image size
        cv2.imshow("Image window", current_frame_contours_small)
        cv2.waitKey(1)


    def timer_callback(self):
        #print('entered timer_callback')

        self.tw=Twist() # twist message to publish

        self.tw.angular.z = self.turnVel

        if(self.searching == True):
            self.tw.linear.x = 0.25
            self.pub_vel.publish(self.tw)

    # This method takes four arguments: the colour to check, the contour to look for the colour in, 
    # and the x and y coordinates of the center of the contour.
    def colourSeen(self, colour, contour, cx, cy):
        
        # Check if the contour has any points
        if len(contour) > 0:
            
            # Check if the contour has enough points to be considered valid (more than 10 in this case)
            if (len(contour[0]) > 10):
                
                # Calculate the moments of the contour using OpenCV
                M2 = cv2.moments(contour[0])

                if M2['m00'] > 0:
                    # Calculate the center of the contour using the moments
                    contour_cx = int(M2['m10']/M2['m00'])
                    contour_cy = int(M2['m01']/M2['m00'])

                    # Check if the center of the contour matches the x and y coordinates passed as arguments
                    if contour_cx == cx and contour_cy == cy:
                        
                        # If the center of the contour matches the arguments, print a message indicating that the colour has been seen
                        print(f"{colour} can be seen infront of the robot")
                        
                        # Set the value of the colourClose dictionary at the key of the current colour to True
                        self.colourClose[colour] = True

        

def main(args=None):
    print('Starting colourChaser.py.')

    rclpy.init(args=args)

    colourChaser = ColourChaser()

    rclpy.spin(colourChaser)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    colourChaser.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()