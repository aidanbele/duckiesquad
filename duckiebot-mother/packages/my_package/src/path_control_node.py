#!/usr/bin/env python3
# Acquired from https://github.com/ethanmusser/velocity-controller/blob/2515e89893181e57a49111df2f85861f4dca5477/src/vel_func_node/scripts/vel_func_node_back.py
import rospy
import sys
import os

from duckietown_msgs.msg import Twist2DStamped


def velocityPublisher(host):
    # Initialize velocity publisher
    # pub = rospy.Publisher('/'+host+'/vel_func_node/car_cmd',Twist2DStamped,queue_size=1)
    pub = rospy.Publisher('/'+host+'/joy_mapper_node/car_cmd',
                          Twist2DStamped, queue_size=1)
    
    # Initialize message
    msg = Twist2DStamped()

    # Define shutdown hook
    # rospy.on_shutdown(shutdown_hook)

    # Define publish rate
    rate = rospy.Rate(1)  # 1hz

    # Publish at defined rate until user keyboard interrupt
    print("CTRL + C to stop motors.")
    while not rospy.is_shutdown():
        try:
            rospy.loginfo("Publshing Velocities")
            msg.header.stamp = rospy.Time.now()
            msg.v = 0.1
            msg.omega = 0.0
            pub.publish(msg)
            rate.sleep()
        except rospy.ROSInterruptException:
            rospy.loginfo("Shutdown Initiated: Stopping motors")
            msg.header.stamp = rospy.Time.now()
            msg.v = 0.0
            msg.omega = 0.0
            pub.publish(msg)
            rospy.sleep(1)
            break

# def shutdown_hook():
#     pub = rospy.Publisher('/duckiebot02/joy_mapper_node/car_cmd',Twist2DStamped,queue_size=1)
#     print("Shutdown Initiated: Stopping motors.")
#     msg = Twist2DStamped()
#     msg.header.stamp = rospy.Time.now()
#     msg.v = 0.0
#     msg.omega = 0.0
#     pub.publish(msg)
#     rospy.sleep(1)


if __name__ == '__main__':
    # Initialize node
    rospy.init_node('vel_func_node', anonymous=False)
    rospy.loginfo("Starting node %s" % rospy.get_name())

    # Set hostname based on optional vehicle name
    if len(sys.argv) != 2:
        rospy.loginfo(
            "Vehicle name not passed as a command line argument, expected to be passed as ROS variable")
        try:
            hostname = os.environ['VEHICLE_NAME']
        except:
            raise Exception("ROS parameter '~veh' not found!")
    else:
        hostname = sys.argv[1]
    rospy.loginfo('Hostname: %s' % hostname)

    # Call velocityPublisher
    try:
        velocityPublisher(host=hostname)
    except rospy.ROSInterruptException:
        raise Exception(
            "Error encountered when attempting to start vel_func_node velocityPublisher!")