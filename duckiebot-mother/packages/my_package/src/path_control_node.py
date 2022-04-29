#!/usr/bin/env python3

import os
import rospy
from duckietown.dtros import DTROS, NodeType
from duckietown_msgs.msg import Twist2DStamped

class LedControlNode(DTROS):

    def __init__(self, node_name):
        # initialize the DTROS parent class
        super(LedControlNode, self).__init__(node_name=node_name, node_type=NodeType.GENERIC)
        
        self.veh_name = os.environ['VEHICLE_NAME']

        #Setup the wheel publisher
        car_topic=f"/{ self.veh_name }/joy_mapper_node/car_cmd"
        self.car = rospy.Publisher(car_topic, Twist2DStamped, queue_size=1)
        

    def run(self):

        cmds = [[0.5, 0], [0.5, 1], [0.5, 1], [0.5, 1]]

        rate = rospy.Rate(0.2) # 1Hz
        while not rospy.is_shutdown():
            for cmd in cmds:
                if not rospy.is_shutdown():
                    self.car.publish(self.createCarCmd(cmd[0], cmd[1]))
                    rate.sleep()
            


    def createCarCmd(self,v,o):
        msg = Twist2DStamped()
        msg.header.stamp = rospy.Time.now()
        msg.v = v
        msg.omega = o
        return msg


    def onShutdown(self):
        """Shutdown procedure.
        Publishes a zero velocity command at shutdown."""

        self.car.publish(self.createCarCmd(0, 0))

        super(LedControlNode, self).onShutdown()


if __name__ == '__main__':
    # create the node
    node = LedControlNode(node_name='led_control_node')
    # run node
    node.run()
    node.onShutdown()
    # keep spinning
    rospy.spin()
