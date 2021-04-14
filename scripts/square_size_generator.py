#!/usr/bin/env python

#####################################################################################################

# Code Developed by Raja Chellappan
# Student ID: 200716420
# Queen Mary University of London

# Square Size Generator is used to create a random value using Python.random.uniform() function
# for the size of the square every 20 seconds 
# the length of the side of the square should be random real number between 0.05 and 0.20

#####################################################################################################

import rospy
import random

from ar_week10_test.msg import square_size

#########################################################################################################

def square_size_generator():
    
    #Topic initialization
    pub = rospy.Publisher('size', square_size, queue_size=0)
    
    #New Node initialization
    rospy.init_node('square_size_generator', anonymous=True)
    
    #Generate every 20 seconds (0.05hz = 20s)
    rate = rospy.Rate(0.05)
    
    msg = square_size()
    while not rospy.is_shutdown():
    	
    	#Generate random real number between 0.05 and 20
        msg.size = random.uniform(0.05, 0.20)
        
        rospy.loginfo(msg)
        pub.publish(msg)
        rate.sleep()

#########################################################################################################

#########################################################################################################

if __name__ == '__main__':
    try:
        square_size_generator()
    except rospy.ROSInterruptException: #Exception Handling
        pass

#########################################################################################################
