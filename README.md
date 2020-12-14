# exp_assignment2
URDF Foulder :


1) the robot.xacro was modified with macros :

- dded che macros for dimension for the head (box with x_head,y_head,z_head variables)
-added che macros for dimension for the chassis (box with x_chassis,y_chassis,z_chassis variables)
-added che macros for dimension for the neck (cylinder with length_neck,radius_neck variables)

-created come macros for the inertia with the variable on input to calculate for box and cylinder

2) the robot.xacro was modified  with link and joints:

-added the link of the head and the neck
-added the joint of the head and the neck (initaly all two was fixed after i will change the head to be a revolute motion in z )

3) the robot.gazebo 

-was added the material for the gazebo simulation 
-was added after the plugin to controll the head movement

SCRIPT Foulder : 

Created 2 new python file : 

1)fsm.py :
Was the smach state machine of ass1 completly modified addind :
State Normal: 
-



