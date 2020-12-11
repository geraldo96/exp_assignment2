#!/usr/bin/env python 

import rospy
from fsm_receiver.msg import Command
from position_receiver.msg import Position
import smach
import smach_ros
from smach import CBState
import time
import random
import roslib
from math import atan2

from geometry_msgs.msg import Twist,Point
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion

x = 0.0
y = 0.0
theta = 0.0
flag_ball_detected = 0
flag_home = 0


def newOdom(msg):
	global x
	global y
	global theta

	x = msg.pose.pose.position.x
	y  = msg.pose.pose.position.y
	tor_q = msg.pose.pose.orientation
	(roll, pitch, theta) = euler_from_quaternion ([rot_q.x,rot_q.y, rot_q.z, rot_q.w])

		



@smach.cb_interface(input_keys=[], output_keys=[], outcomes=['rep_normal','sleep'])
    # define state Normal
def normal_cb(user_data):
    #--- Code to move randomly ---
    
	    #rospy.init_node('controller_normal_randomness', anonymous=True)
      pub = rospy.Publisher('/robot/cmd_vel', Twist, queue_size=10)

      movement_cmd = Twist()
      r = rospy.Rate(10) #10hz


      if flag_ball_detected < 10:
          movement_cmd.linear.x =(random.randint(0, 20))/10
          movement_cmd.angular.z =random.randint(-2, 2)
          rospy.loginfo('Sto dando valori random, dovrei muovermi')
          pub.publish(movement_cmd)
          r.sleep()
          time.sleep(2)
          flag_ball_detected=+1
          rospy.loginfo(flag_ball_detected)
          return 'rep_normal'

      else:
          return 'sleep'
          #time.sleep(20) #after reaching the home he sleep for 20 sec.



@smach.cb_interface(input_keys=[], output_keys=[], outcomes=['rep_sleep','normal'])
    # define state Sleep
def sleep_cb(user_data):
    #--- Code to move at home ---
      
      #rospy.init_node ("home_controller")
      sub_odom = rospy.Subscriber("/robot/odom",Odometry, newOdom)
      pub_vel = rospy.Subscriber("/robot/cmd_vel",Twist, queue_size=10) 

      speed = Twist()
      r = rospy.Rate(10)

      goal = Point()
      goal.x = -5 #current home position x
      goal.y = 5 #current home position y

      if flag_home == 0:
          inc_x = goal.x - x
          inc_y = goal.y - y

          angle_to_goal = atan2(inc_y, inc_x)

          if abs(angle_to_goal - theta) > 0.1:
            speed.linear.x = 0.0
            speed.angular.z = 0.3
          else:
            speed.linear.x = 0.5
            speed.angular.z = 0.0
          if abs(inc_x)<0.1 & abs(inc_y)<0.1:
            speed.linear.x = 0.0
            speed.angular.z = 0.0
            flag_home=1

          pub.publish(speed)
          r.sleep()
          return 'rep_sleep'

      else:
          time.sleep(20) #after reaching the home he sleep for 20 sec.
          return 'normal'
          


	

        
if __name__ == '__main__':

    rospy.init_node('smach_ass1_state_machine')
    #rospy.init_node('listener', anonymous=True)

    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=['container_interface'])

    # Open the container
    with sm:
        smach.StateMachine.add('NORMAL', CBState(normal_cb), 
                               {'rep_normal':'NORMAL','sleep':'SLEEP'}),
        smach.StateMachine.add('SLEEP', CBState(sleep_cb), 
                               {'rep_sleep':'SLEEP','normal':'NORMAL'}),


    # Execute the state machine
    outcome = sm.execute()
    
