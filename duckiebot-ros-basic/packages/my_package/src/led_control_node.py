#!/usr/bin/env python3

import os
import random
import rospy
from duckietown.dtros import DTROS, NodeType
from duckietown_msgs.msg import LEDPattern
from duckietown_msgs.srv import SetCustomLEDPattern
from duckietown_msgs.msg import WheelsCmdStamped

class LedControlNode(DTROS):

    def __init__(self, node_name):
        # initialize the DTROS parent class
        super(LedControlNode, self).__init__(node_name=node_name, node_type=NodeType.GENERIC)
        
        self.veh_name = os.environ['VEHICLE_NAME']


        #Setup the wheel publisher
        wheel_topic=f"/{self.veh_name}/wheels_driver_node/wheels_cmd"
        self.wheels = rospy.Publisher(wheel_topic, WheelsCmdStamped, queue_size=1)
        

    def run(self):
        # change colors randomly every second
        rate = rospy.Rate(1) # 1Hz
        while not rospy.is_shutdown():
            self.set_LEDs()
            self.wheels.publish(self.createWheelCmd(random.uniform(0.0, 0.5),random.uniform(0.0, 0.5)))
            rate.sleep()

    
    def set_LEDs(self, color_list=None):
        led_service = f"/{self.veh_name}/led_emitter_node/set_custom_pattern"
        # rospy.wait_for_service(led_service)

        if color_list == None:
            color_list = self.random_color_list()
        try:
            service = rospy.ServiceProxy(led_service, SetCustomLEDPattern)
            msg = LEDPattern()
            msg.color_list = color_list
            msg.color_mask = [1, 1, 1, 1, 1]
            msg.frequency = 0
            msg.frequency_mask = [0, 0, 0, 0, 0]
            response = service(msg)
            rospy.loginfo(response)
        except rospy.ServiceException as e:
            print (f"Service call failed: {e}")

    def random_color_list(self):
        # color list does not include "switchedoff" so we are a real party
        color_list = ["green", "red", "blue", "white", "yellow", "purple", "cyan", "pink"]
        return random.choices(color_list, k = 5)
    

    def createWheelCmd(self,left,right):
        wheels_cmd_msg = WheelsCmdStamped()
        # spin right unless servoing or centered
        wheels_cmd_msg.header.stamp = rospy.Time.now()
        wheels_cmd_msg.vel_left = left
        wheels_cmd_msg.vel_right = right
        return wheels_cmd_msg

    def onShutdown(self):
        """Shutdown procedure.
        Publishes a zero velocity command at shutdown."""

        # MAKE SURE THAT THE LAST WHEEL COMMAND YOU PUBLISH IS ZERO,
        # OTHERWISE YOUR DUCKIEBOT WILL CONTINUE MOVING AFTER
        # THE NODE IS STOPPED

        leds_off = ["switchedoff", "switchedoff", "switchedoff", "switchedoff", "switchedoff"]
        self.set_LEDs(leds_off)

        self.wheels.publish(self.createWheelCmd(0.0,0.0))

        super(LedControlNode, self).onShutdown()

if __name__ == '__main__':
    # create the node
    node = LedControlNode(node_name='led_control_node')
    # run node
    node.run()
    # keep spinning
    rospy.spin()
