#!/usr/bin/env python 

import rospy
from fsm_receiver.msg import Command
from position_receiver.msg import Position
import smach
import smach_ros
import time
import random
import roslib

Move_to_play = False

def talker():
    pub = rospy.Publisher('chatter_position', Position, queue_size=10)
    #rospy.init_node('talker_from_fsm', anonymous=True)
    r = rospy.Rate(10) #10hz
    msg = Position()
    msg.x = 4
    msg.y = 4
    i = 0
    while i<10 :
	i = i +1
        rospy.loginfo(msg)
        pub.publish(msg)
        r.sleep()



def callback(data):
     global Move_to_play

     #rospy.loginfo("Mode is :%d ,User is in position: X = %d , Y = %d" % (data.state, data.x, data.y))
     if  data.state == 1:
		Move_to_play=1
     else :
		Move_to_play=0
		
		


def user_action():
	global Move_to_play
	
	rospy.Subscriber("chatter_command", Command, callback)
	if Move_to_play == 1:
		Move_to_play  = 0;
		return 'play'
	else :
		return 'sleep_from_normal'
	
   



    # define state Normal
class Normal(smach.State):
    def __init__(self):
        # initialisation function, it should not wait
        smach.State.__init__(self, 
                             outcomes=['sleep_from_normal','play'],
                             input_keys=['normal_counter_in'],
                             output_keys=['normal_counter_out'])
        
    def execute(self, userdata):
	time.sleep(10)
	
	rospy.loginfo('Executing state NORMAL (users = %f)'%userdata.normal_counter_in)
	userdata.normal_counter_out = userdata.normal_counter_in + 1 
	return user_action()



# define state Sleep
class Sleep(smach.State):
    def __init__(self):
        
        smach.State.__init__(self, 
                             outcomes=['normal_from_sleep'],
                             input_keys=['sleep_counter_in'],
                             output_keys=['sleep_counter_out'])
        
    def execute(self, userdata):
	time.sleep(10)
	
	rospy.loginfo('Executing state SLEEP (users = %f)'%userdata.sleep_counter_in)
	userdata.sleep_counter_out = userdata.sleep_counter_in + 1 
	return 'normal_from_sleep'

	

       
# define state Play
class Play(smach.State):
    def __init__(self):
        smach.State.__init__(self, 
                             outcomes=['normal_from_play'],
                             input_keys=['play_counter_in'],
                             output_keys=['play_counter_out'])
        
	self.sensor_input = 0
        self.rate = rospy.Rate(200)  # Loop at 200 Hz

    def execute(self, userdata):
	
        # simulate we recived the play command
        while not rospy.is_shutdown():  
            time.sleep(1)
	    if self.sensor_input < 5: 
		rospy.loginfo('Executing state PLAY (users = %f)'%userdata.play_counter_in)
		userdata.play_counter_out = userdata.play_counter_in + 1
		talker() 
		return 'normal_from_play'
	    self.sensor_input += 1 
	    self.rate.sleep
	

        
def main():
    rospy.init_node('smach_ass1_state_machine')
    #rospy.init_node('listener', anonymous=True)

    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=['container_interface'])
    sm.userdata.sm_counter = 0

    # Open the container
    with sm:
        # Add states to the container
	smach.StateMachine.add('NORMAL', Normal(), 
                               transitions={'sleep_from_normal':'SLEEP', 
                                            'play':'PLAY'},
                               remapping={'normal_counter_in':'sm_counter',
                                          'normal_counter_out':'sm_counter'})

        smach.StateMachine.add('SLEEP', Sleep(), 
                               transitions={'normal_from_sleep':'NORMAL'},
                               remapping={'sleep_counter_in':'sm_counter', 
                                          'sleep_counter_out':'sm_counter'})
        
	smach.StateMachine.add('PLAY', Play(), 
                               transitions={'normal_from_play':'NORMAL'},
                               remapping={'play_counter_in':'sm_counter',
                                          'play_counter_out':'sm_counter'})


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
