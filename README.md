# exp_assignment2
URDF Foulder :


1) the robot.xacro was modified with macros :

- added che macros for dimension for the head (box with x_head,y_head,z_head variables)
- added che macros for dimension for the chassis (box with x_chassis,y_chassis,z_chassis variables)
- added che macros for dimension for the neck (cylinder with length_neck,radius_neck variables)

- created come macros for the inertia with the variable on input to calculate for box and cylinder

2) the robot.xacro was modified  with link and joints:

- added the link of the head and the neck
- added the joint of the head and the neck (initaly all two was fixed after i will change the head to be a revolute motion in z )

3) the robot.gazebo 

- was added the material for the gazebo simulation 
- was added after the plugin to controll the head movement

SCRIPT Foulder : 

Created 2 new python file : 

1) fsm.py , contain the state machine from smach with:
- NORMAL State , send random value to the /robot/cmd_vel and if detect the ball move to State PLAY;
- PLAY State , follow the detected ball untill the ball disapear and move to State SLEEP;
- SLEEP State , Move to the Home position but is bad implemente because the position is given from the odom because a word frame is not used, but wait some sec to the home and after move to State NORMAL;

2) cv.py , contain the OpenCV operation , if detect something publish to /robot/balldetec and the fsm.py sub to the topic to change the state to PLAY,and recive another flag from the fsm.py to start to follow the ball.

Problem with Debug:
- There are some commont part to control the join of the head,i could not implement it perfectly then i removed : revolute to fixed on .xacro file , trasmition component, config foulder and some line to launch the controll for the joint.







