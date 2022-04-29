#!/usr/bin/env python3
# Acquired from https://github.com/Duckietown-NCTU/Software/blob/59a55c5d7453fc2c45c1686eb6958bfa0d5d8ab9/catkin_ws/src/dagu_car/script/dagu_differential_drive_test.py

import rospy
from duckietown_msgs.msg import CarControl

if __name__ == '__main__':
    # Initialize the node with rospy
    rospy.init_node('dagu_car_tester', anonymous=False)
    pub = rospy.Publisher("~car_control",CarControl,queue_size=1)
    rospy.loginfo("[dagu_car_tester] Initializing.")
    rospy.sleep(1.0)

    # List of tuples in the form (speed, steering, sleep_time)
    command_list = [(1.0, 0.0, 1), (1.0, 1.0, 3)]
    # Loop 4 times to go in a square
    for i in range(4):
        for j in range(len(command_list)):
            pub.publish(CarControl(speed=command_list[j][0],steering=command_list[j][1]))
            if j == 0:
                rospy.loginfo("[dagu_car_tester] Full speed ahead.")
            else:
                rospy.loginfo("[dagu_car_tester] Full speed left turn.")
            rospy.sleep(command_list[j][2])
    
    # Full speed ahead
    """ pub.publish(CarControl(speed=1.0,steering=0.0))
    rospy.loginfo("[dagu_car_tester] Full speed ahead.")
    rospy.sleep(3.0)
    # Full speed reverse
    pub.publish(CarControl(speed=-1.0,steering=0.0))
    rospy.loginfo("[dagu_car_tester] Full speed reverse")
    rospy.sleep(3.0)
    # Full speed left turn
    pub.publish(CarControl(speed=1.0,steering=1.0))
    rospy.loginfo("[dagu_car_tester] Full speed left turn")
    rospy.sleep(3.0)
    # Full speed left turn
    pub.publish(CarControl(speed=1.0,steering=-1.0))
    rospy.loginfo("[dagu_car_tester] Full speed right turn")
    rospy.sleep(3.0) """
    # Stop
    pub.publish(CarControl(speed=0.0,steering=0.0))
    rospy.loginfo("[dagu_car_tester] Stop")
    rospy.sleep(3.0)

    rospy.loginfo("[dagu_car_tester] Testing Done")
    # rospy.spin()