#!/usr/bin/env python

# Python libs
import sys
import time

# numpy and scipy
import numpy as np
from scipy.ndimage import filters

import imutils

# OpenCV
import cv2

# Ros libraries
import roslib
import rospy

# Ros Messages
from std_msgs.msg import Bool
from sensor_msgs.msg import CompressedImage
from geometry_msgs.msg import Twist

VERBOSE = False
Flag_play = False


class image_feature:

    def __init__(self):
        '''Initialize ros publisher, ros subscriber'''
        rospy.init_node('image_feature', anonymous=True)
        # topic where we publish
        self.image_pub = rospy.Publisher("/output/image_raw/compressed",
                                         CompressedImage, queue_size=1)
        self.det_pub = rospy.Publisher("robot/detectblob",Bool, queue_size=1)
        self.vel_pub = rospy.Publisher("robot/cmd_vel",Twist, queue_size=1)

        # subscribed Topic
        self.subscriber = rospy.Subscriber("robot/camera1/image_raw/compressed",
                                           CompressedImage, self.callback,  queue_size=1)

        self.det_state = rospy.Subscriber("robot/state",Bool, self.playcallback,  queue_size=1)


    def playcallback(self,data):
     global Flag_play

     if  data.data == True:
        #rospy.loginfo('Flag_play == True')
        Flag_play=True
     if  data.data == False:
        #   rospy.loginfo('Flag_play == False')
        Flag_play=False


    def callback(self, ros_data):
        '''Callback function of subscribed topic. 
        Here images get converted and features detected'''
        if VERBOSE:
            print ('received image of type: "%s"' % ros_data.format)

        #### direct conversion to CV2 ####
        np_arr = np.fromstring(ros_data.data, np.uint8)
        image_np = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)  # OpenCV >= 3.0:

        greenLower = (50, 50, 20)
        greenUpper = (70, 255, 255)

        blurred = cv2.GaussianBlur(image_np, (11, 11), 0)
        hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, greenLower, greenUpper)
        mask = cv2.erode(mask, None, iterations=2)
        mask = cv2.dilate(mask, None, iterations=2)
        #cv2.imshow('mask', mask)
        cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL,
                                cv2.CHAIN_APPROX_SIMPLE)
        cnts = imutils.grab_contours(cnts)
        center = None

        # only proceed if at least one contour was found
        if (len(cnts) > 0) :
            self.det_pub.publish(True) #send to fsm the detection of something
        else:
            self.det_pub.publish(False) #send to fsm the detection of nothing

        if (Flag_play==True):
            rospy.loginfo('seguo')
            c = max(cnts, key=cv2.contourArea)
            ((x, y), radius) = cv2.minEnclosingCircle(c)
            M = cv2.moments(c)
            center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))

            # only proceed if the radius meets a minimum size
            if radius > 10:
                # draw the circle and centroid on the frame,
                # then update the list of tracked points
                rospy.loginfo('updating')
                cv2.circle(image_np, (int(x), int(y)), int(radius),
                           (0, 255, 255), 2)
                cv2.circle(image_np, center, 5, (0, 0, 255), -1)
                vel = Twist()
                vel.angular.z = -0.002*(center[0]-400)
                vel.linear.x = -0.01*(radius-100)
                self.vel_pub.publish(vel)
            else:
                rospy.loginfo('dritto')
                vel = Twist()
                vel.linear.x = 0.5
                self.vel_pub.publish(vel)

    

        cv2.imshow('window', image_np)
        cv2.waitKey(2)


def main(args):
    '''Initializes and cleanup ros node'''
    ic = image_feature()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print ("Shutting down ROS Image feature detector module")
    cv2.destroyAllWindows()


if __name__ == '__main__':
    main(sys.argv)
