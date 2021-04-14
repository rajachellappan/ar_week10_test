--------------------------------------------------------------------------------------------------------------------
Code Developed by Raja Chellappan
Student ID: 200716420
Queen Mary University of London
--------------------------------------------------------------------------------------------------------------------


###################################################################################################################

Follow the steps to run the package:

STEP 1: Download and run the following repositories to run rViz with the Panda Arm Robot in your catkin workspace
	
	$ git clone -b ROS-DISTRO-devel https://github.com/ros-planning/moveit_tutorials.git
	$ git clone -b ROS-DISTRO-devel https://github.com/ros-planning/panda_moveit_config.git
	
	where ROS-DISTRO is the name of the ROS Distribution (eg. melodic, noetic etc.)
	
STEP 2: Unzip the ar_week10_test.zip folder in the src folder of your catkin workspace;

STEP 3: Build the catkin workspace and source it to the setup
	
	$ catkin_make
	$ source ~/catkin_ws/devel/setup.bash
	
STEP 4: Make all the nodes executable by running the following commands in the "scripts" folder 

	$ chmod +x square_size_generator.py
	$ chmod +x move_panda_square.py

STEP 5: Run the Roscore

	$ roscore
	
STEP 6: Run the following (on four different terminals);

	$ roslaunch panda_moveit_config demo.launch
	$ rosrun ar_week10_test square_size_generator.py
	$ rosrun ar_week10_test move_panda_square.py
	$ rosrun rqt_gui rqt_gui
	
######################################################################################################################
