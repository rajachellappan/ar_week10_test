#!/usr/bin/env python

#####################################################################################################

# Code Developed by Raja Chellappan
# Student ID: 200716420
# Queen Mary University of London

# This node subscribes to the ROS topic square_size_generator, waits for the messages
# when a new message is received it does the following
# 1. Move the panda robot to a starting configuration
# 2. Plan a Carteisan path
# 3. Show the planned trajectory
# 4. Execute the planned trajectory
# 5. Wait for the next message

#####################################################################################################

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import time
from math import pi

from ar_week10_test.msg import square_size

#####################################################################################################

def callback(data):
    
    try:
        
        
        # Receive the random square size from the square_size_generator
        print('-------------------------------------------------------------------------------')
        print ('Move Panda - Received square size, s = %s ' % data.size)
        print('-------------------------------------------------------------------------------')
        
        # Set the Starting configuration
        print('-------------------------------------------------------------------------------')
        print ('Move Panda - Going to Start Configuration')
        print('-------------------------------------------------------------------------------')
        start_conf = [0, -pi / 4, 0, -pi / 2, 0, pi / 3, 0]

        #Set the Panda arm to the given start configuration
        group.go(start_conf, wait=True)

        #Calling ``stop()`` ensures that there is no residual movement
        group.stop()

		#Planning Motion Trajectory
        print('-------------------------------------------------------------------------------')
        print ('Move Panda - Planning Motion Trajectory')
        print('-------------------------------------------------------------------------------')
        
        
        # Waypoints initialization
        waypoints = []

        #Get the current group positions
        wpose = group.get_current_pose().pose

        # First move sideways (y)
        wpose.position.y += data.size  
        waypoints.append(copy.deepcopy(wpose))

		# Second move forward/backwards in (x)
        wpose.position.x += data.size  
        waypoints.append(copy.deepcopy(wpose))

		# Third move sideways (y)
        wpose.position.y -= data.size  
        waypoints.append(copy.deepcopy(wpose))

        # Forth move forward/backwards in (x)
        wpose.position.x -= data.size  
        waypoints.append(copy.deepcopy(wpose))

        (plan, fraction) = group.compute_cartesian_path(
            waypoints,  # waypoints to follow
            0.01,  # eef_step 
            0.0)  # jump_threshold 

        # Displaying a trajectory
        display_trajectory = moveit_msgs.msg.DisplayTrajectory()
        display_trajectory.trajectory_start = robot.get_current_state()
        display_trajectory.trajectory.append(plan)
        
        # Showing Planned Trajectory
        print('-------------------------------------------------------------------------------')
        print ('Move Panda - Showing Planned Trajectory')
        print('-------------------------------------------------------------------------------')
        display_trajectory_publisher.publish(display_trajectory)

        time.sleep(5)
        
        
        # Executing planned trajectory
        print('-------------------------------------------------------------------------------')
        print ('Move Panda - Executing Planned Trajectory')
        print('-------------------------------------------------------------------------------')
        group.execute(plan, wait=True)

    except (rospy.ServiceException, e):
        print("Service call failed: %s" % e)
########################################################################################################


########################################################################################################
def move_panda_square():
    
    # moveit_commander initialization
    moveit_commander.roscpp_initialize(sys.argv)

    # new node initialization
    rospy.init_node('move_panda_square', anonymous=True)

    # Waiting for desired size of square trajectory
    print('-------------------------------------------------------------------------------')
    print ('Move Panda - Waiting for desired size of square trajectory')
    print('-------------------------------------------------------------------------------')
    
    # subscribe to cubic_traj_params and send data to callback
    rospy.Subscriber('size', square_size, callback)
    
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()
########################################################################################################

########################################################################################################
if __name__ == "__main__":
    
    # Panda Arm robot moveit_commander initialization
    robot = moveit_commander.RobotCommander()

    # Panda Arm robot moveit Planning Scene Interface initialization
    scene = moveit_commander.PlanningSceneInterface()

    # Panda Arm robot moveit Move Group Commander intialization
    group = moveit_commander.MoveGroupCommander('panda_arm')

    # Panda Arm robot moveit Display trajectory publisher initialization
    display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                                   moveit_msgs.msg.DisplayTrajectory,
                                                   queue_size=0)
                                          
                                                   
    move_panda_square()

########################################################################################################         
