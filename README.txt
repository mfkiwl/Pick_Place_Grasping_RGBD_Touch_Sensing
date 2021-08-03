@Author Hyungjoo Kim, KaKei Choi, Youseff Al Jrab
@Title Autonomous Pick and Place for Grasping Task using RGB-D and Touch Bumper Sensing

Pre-requisites:
> sudo apt-get install ros-melodic-pcl-conversions
> cd ws_comp0129/src
> rm -rf moveit_tutorials
> git clone -b melodic-devel https://github.com/RPL-CSUCL/moveit_tutorials.git
> git clone https://github.com/HJKIMRobotics/Pick_Place_Grasping_RGBD_Touch_Sensing

Run:
> source devel/setup.bash
> roslaunch cw3 pick_and_place.launch


Question 1.1 - "filterEnvironment" function in cylinder_segment.cpp file
The normal framerate is approximately 1.5 Hz.
When 'f' is pressed, you can check a improvement of the framerate approx. 11 Hz.

Question 1.2 - "fastFilterEnvironment" function in cylinder_segment.cpp file
The normal framerate is approximately 1.5 Hz.
When 'p' is pressed, you can check a segmentation code faster, approx. 15 Hz.

Question 1.3 - "cylinderPosePublish" function in Pick&Place.cpp file
In the rviz, you need to add "Pose" and set the 'cylinder_pose' in topic section.


(!!!REMINDER!!! When '1', '2', '3' are pressed, the message sometimes shows a collision problem and the robot manipulator grasps the objects properly and stops.   Please ignore this message and press '1', '2' or '3' one more time. If you press it one more time or several times until the manipulator is moving, then all tasks work well)
                
Question 2.1 - "cw3Q2BumperPub" function in Pick&Place.cpp file
If you want to check the updated bumper in real-time, you need to run the line 430, 'ROS_INFO_STREAM("bump1:"<<this->bump1_<<" bump2:"<<this->bump2_<<" bump3:"<<this->bump3_);'.

Question 2.2 - "cw3Q2Grasp1" function in Pick&Place.cpp file
Firstly, the cylinder object locates on the ground.
When '1' is pressed, the manipulator grasps it and places on table 1.
Also, you can check the updated bumper in real-time if you run the line 430.

Question 3 - "checkAvailable", "cw3Q3GraspObject", "cw3Q3Grasp2" and "cw3Q3Grasp3" function in Pick&Place.cpp file
i) By pressing '1': If the bumper sensor from table 1 is detected, then check the bumper sensor from table 2. If the bumper sensor from table 2 is also detected, then the manipulator grasps the box-object and places it on table 3, otherwise place it on table 2. After that, run to place the cylinder object on table 1.

ii) By pressing '2': If the bumper sensor from table 2 is detected, then check the bumper sensor from table 1. If the bumper sensor from table 1 is also detected, then the manipulator grasps the box-object and places it on table 3, otherwise place it on table 1. After that, run to place the cylinder object on table 2.

iii) By pressing '3': If the bumper sensor from table 3 is detected, then check the bumper sensor from table 2. If the bumper sensor from table 2 is also detected, then the manipulator grasps the box-object and places it on table 1, otherwise place it on table 2. After that, run to place the cylinder object on table 3.

