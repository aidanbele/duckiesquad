#!/usr/bin/env python3

import os
import random
import numpy as np
import cv2 as cv
import time
import rospy
from duckietown.dtros import DTROS, NodeType
from duckietown_msgs.msg import LEDPattern, Twist2DStamped
from duckietown_msgs.srv import SetCustomLEDPattern
from sensor_msgs.msg import CompressedImage
from duckietown_utils.jpg import bgr_from_jpg
from cv_bridge import CvBridge

# homography and computer vision code based off:
# https://github.com/charan223/charan_ros_core/blob/v1/packages/purepursuit/src/purepursuit_controller_node.py
# https://github.com/duckietown-ethz/proj-lfvop/blob/master/packages/dynamic_obstacle_avoidance/src/duckie_detection_node.py


RANDOM_COLORS = ["green", "red", "blue", "white", "yellow", "purple", "cyan", "pink"]
bridge = CvBridge()

class MotherControlNode(DTROS):

    def __init__(self, node_name):
        # initialize the DTROS parent class
        super(MotherControlNode, self).__init__(node_name=node_name, node_type=NodeType.GENERIC)
        
        rospy.loginfo(os.environ['VEHICLE_NAME'])
        self.veh_name = os.environ['VEHICLE_NAME']

        # Setup the wheel publisher
        car_topic=f"/{self.veh_name}/joy_mapper_node/car_cmd"
        self.car = rospy.Publisher(car_topic, Twist2DStamped, queue_size=1)

        # Setup the image subscriber
        ing_topic = f"/{self.veh_name}/camera_node/image/compressed"
        rospy.Subscriber(ing_topic, CompressedImage, callback=self.processImage, queue_size=1)

        # kinematics of car
        self.v = 0
        self.omega = 0

    def run(self):
        # change colors randomly every second
        leds_on = ["green", "green", "green", "green", "green"]
        self.set_LEDs(leds_on)
        rate = rospy.Rate(2) # run twice every second
        while not rospy.is_shutdown():
            # multiply speed times 1.7 because it runs a bit better on carpet at this speed
            self.car.publish(self.createCarCmd(self.v * 1.7, self.omega))
            self.set_LEDs(leds_on)
            rate.sleep()


    def processImage(self, image_msg):
        image_size = [120,160]
        # top_cutoff = 40

        #rospy.loginfo("Start processing image")

        start_time = time.time()
        try:
            image_cv = bgr_from_jpg(image_msg.data)
        except ValueError as e:
            rospy.loginfo("image decode error", e)
            return
        
        # Resize and crop image
        hei_original, wid_original = image_cv.shape[0:2]

        if image_size[0] != hei_original or image_size[1] != wid_original:
            image_cv = cv.resize(image_cv, (image_size[1], image_size[0]), interpolation=cv.INTER_NEAREST)

        # uses HSV not RGB - we use a bitwise or here because the red color specturm wraps around
        hsv = cv.cvtColor(image_cv, cv.COLOR_BGR2HSV)
        lower_black = np.array([0, 0, 0])
        upper_black = np.array([350,55,100])

        mask = cv.inRange(hsv, lower_black, upper_black)

        contours = cv.findContours(mask.copy(), cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)[-2]

        if len(contours)>1:
            duckie_area = max(contours, key=cv.contourArea)
            (x,y,w,h) = cv.boundingRect(duckie_area)
            area = w * h
            rospy.loginfo(f"X: {[x, x+w]}, Y: {[y, y+h]}, w: {w}, h: {h}, area: {area}")

            if area < 10 or area > 10000 or y < 10:
                self.v = -0.2
                self.omega = 0
                rospy.loginfo("BACKWARDS")
            elif w / h > 1.5:
                self.v = 0.1
                self.omega = -3 # always try turning right
                rospy.loginfo("HARD RIGHT")
            else:
                self.v = 0.2
                self.omega = -max(min((90 - ((2 * x + w) / 2)) / 20, 2), -2) # steer in opposite direction of blob
                rospy.loginfo(f"SLOW, omega: {self.omega}")
        else:
            rospy.loginfo(f"FOUND NOTHING, MAINTING. v: f{self.v}, omega: {self.omega}")
    
    def set_LEDs(self, color_list=None):
        led_service = f"/{self.veh_name}/led_emitter_node/set_custom_pattern"
        # rospy.wait_for_service(led_service)

        if color_list == None:
            color_list = random.choices(RANDOM_COLORS, k = 5)
        try:
            service = rospy.ServiceProxy(led_service, SetCustomLEDPattern)
            msg = LEDPattern()
            msg.color_list = color_list
            msg.color_mask = [1, 1, 1, 1, 1]
            msg.frequency = 0
            msg.frequency_mask = [0, 0, 0, 0, 0]
            service(msg)
        except rospy.ServiceException as e:
            print (f"LED Service call failed: {e}")
    

    def createCarCmd(self, v, omega):
        msg = Twist2DStamped()
        msg.header.stamp = rospy.Time.now()
        msg.v = v
        msg.omega = omega
        return msg

    
    def onShutdown(self):
        """Shutdown procedure.
        Publishes a zero velocity command at shutdown."""

        # MAKE SURE THAT THE LAST WHEEL COMMAND YOU PUBLISH IS ZERO,
        # OTHERWISE YOUR DUCKIEBOT WILL CONTINUE MOVING AFTER
        # THE NODE IS STOPPED

        self.car.publish(self.createCarCmd(0, 0))

        leds_off = ["switchedoff", "switchedoff", "switchedoff", "switchedoff", "switchedoff"]
        self.set_LEDs(leds_off)

        #super(MotherControlNode, self).onShutdown()

if __name__ == '__main__':
    # create the node
    node = MotherControlNode(node_name='daughter_control_node')
    # run node
    node.run()
    node.onShutdown()
    # keep spinning
    rospy.spin()
