#!/usr/bin/env python 

import rospy
import smach
import smach_ros
import time
import random
import roslib
from math import atan2
from std_msgs.msg import Bool
import numpy as np
from scipy.ndimage import filters
import cv2
import imutils

from geometry_msgs.msg import Twist,Point
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from sensor_msgs.msg import CompressedImage


x = 0.0
y = 0.0
theta = 0.0
Move_to_play = False





def callback(data):
     global Move_to_play

     
     if  data.data == True:
        Move_to_play=True
     else :
        Move_to_play=False



def newOdom(msg):
	global x
	global y
	global theta

	x = msg.pose.pose.position.x
	y  = msg.pose.pose.position.y
	rot_q = msg.pose.pose.orientation
	(roll, pitch, theta) = euler_from_quaternion ([rot_q.x,rot_q.y, rot_q.z, rot_q.w])

		


    # define state Normal
class Normal(smach.State):
    def __init__(self):
        # initialisation function, it should not wait
        smach.State.__init__(self, 
                             outcomes=['play','sleep'],
                             input_keys=['normal_counter_in'],
                             output_keys=['normal_counter_out'])
        
    def execute(self, userdata):

            #--- Code to move randomly ---
    
	    
	    mov_pub = rospy.Publisher('/robot/cmd_vel', Twist, queue_size=10)
            movement_cmd = Twist()
	    
            r = rospy.Rate(10) #10hz

            rospy.Subscriber("robot/detectblob",Bool, callback)

            

            while Move_to_play == False:

                rospy.Subscriber("robot/detectblob",Bool, callback)
            	movement_cmd.linear.x =(random.randint(0, 15))/10
            	movement_cmd.angular.z =random.randint(-2, 2)
            	
            	mov_pub.publish(movement_cmd)
            	r.sleep()

            movement_cmd.linear.x = 0
            movement_cmd.angular.z = 0
            mov_pub.publish(movement_cmd)
            rospy.loginfo('Trovato Qualcosa : Move to PLAY')
            return 'play'



# define state Sleep
class Sleep(smach.State):
    def __init__(self):
        
        smach.State.__init__(self, 
                             outcomes=['normal'],
                             input_keys=['sleep_counter_in'],
                             output_keys=['sleep_counter_out'])
        
    def execute(self, userdata):

    	
    	sub_odom = rospy.Subscriber("/robot/odom",Odometry, newOdom)
    	pub_vel = rospy.Publisher("/robot/cmd_vel",Twist, queue_size=10) 

    	speed = Twist()
    	r = rospy.Rate(10)

    	goal = Point()
    	goal.x = 5 #current home position x
    	goal.y = 5 #current home position y
    	flag_home=0

    	while flag_home == 0:
    		inc_x = goal.x - x
    		inc_y = goal.y - y

    		angle_to_goal = atan2(inc_y, inc_x)

    		if abs(angle_to_goal - theta) > 0.1:
    			speed.linear.x = 0.0
    			speed.angular.z = 1.0
    		else:
    			speed.linear.x = 0.5
    			speed.angular.z = 0.0
    		if (abs(inc_x)<0.1) & abs(inc_y)<0.1 :
    			speed.linear.x = 0.0
    			speed.angular.z = 0.0
    			flag_home=1

    		pub_vel.publish(speed)
    		r.sleep()


		time.sleep(20) #after reaching the home he sleep for 20 sec.
		return 'normal'

	

       
# define state Play
class Play(smach.State):
    def __init__(self):
        smach.State.__init__(self, 
                             outcomes=['sleep'],
                             input_keys=['play_counter_in'],
                             output_keys=['play_counter_out'])
        

    

    def execute(self, userdata):
        sub_state = rospy.Publisher('/robot/state', Bool, queue_size=1)
        while Move_to_play == True:

            sub_state.publish(True)
            rospy.Subscriber("robot/detectblob",Bool, callback)
            
        return 'sleep'
            
	

        
def main():
    rospy.init_node('smach_ass1_state_machine')
    #rospy.init_node('listener', anonymous=True)

    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=['container_interface'])
    sm.userdata.sm_ball_detect = 0

    # Open the container
    with sm:
        # Add states to the container
	smach.StateMachine.add('NORMAL', Normal(), 
                               transitions={'play':'PLAY','sleep':'SLEEP'},
                               remapping={'normal_counter_in':'sm_ball_detect',
                                          'normal_counter_out':'sm_ball_detect'})

        smach.StateMachine.add('SLEEP', Sleep(), 
                               transitions={'normal':'NORMAL'},
                               remapping={'sleep_counter_in':'sm_ball_detect', 
                                          'sleep_counter_out':'sm_ball_detect'})
        
	smach.StateMachine.add('PLAY', Play(), 
                               transitions={'sleep':'SLEEP'},
                               remapping={'play_counter_in':'sm_ball_detect',
                                          'play_counter_out':'sm_ball_detect'})


    # Create and start the introspection server for visualization
    sis = smach_ros.IntrospectionServer('server_name', sm, '/SM_ROOT')
    sis.start()

    # Execute the state machine
    outcome = sm.execute()

    # Wait for ctrl-c to stop the application
    rospy.spin()
    sis.stop()


if __name__ == '__main__':
    main()
