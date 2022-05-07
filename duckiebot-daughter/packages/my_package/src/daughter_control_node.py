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
from cv_bridge import CvBridge

# homography and computer vision code based off:
# https://github.com/charan223/charan_ros_core/blob/v1/packages/purepursuit/src/purepursuit_controller_node.py
# https://github.com/duckietown-ethz/proj-lfvop/blob/master/packages/dynamic_obstacle_avoidance/src/duckie_detection_node.py


RANDOM_COLORS = ["green", "red", "blue", "white", "yellow", "purple", "cyan", "pink"]

class DaughterControlNode(DTROS):

    def __init__(self, node_name):
        # initialize the DTROS parent class
        super(DaughterControlNode, self).__init__(node_name=node_name, node_type=NodeType.GENERIC)
        self.veh_name = os.environ['VEHICLE_NAME']

        self.bridge = CvBridge()
        self.resolution=[120,160]
        # self.H = self.load_homography()

        # Setup the wheel publisher
        car_topic=f"/{self.veh_name}/joy_mapper_node/car_cmd"
        self.car = rospy.Publisher(car_topic, Twist2DStamped, queue_size=1)

        # Setup the image subscriber
        img_topic = f"/{self.veh_name}/camera_node/image/compressed"
        rospy.Subscriber(img_topic, CompressedImage, callback=self.processImage, queue_size=1)

        # kinematics of car
        self.v = 0
        self.omega = 0
        self.spin_count = 0
        self.drive_count = 0
        self.green_low = np.array([35,100,100])
        self.green_high = np.array([80,255,255])
        self.unseen_count = 0
        self.leds = ["white", "white", "white", "white", "white"]
        self.prev_leds = []

    def run(self):
        rate = rospy.Rate(2) # run twice every second
        while not rospy.is_shutdown():
            # multiply speed by 1.2 so it runs better on carpet
            self.car.publish(self.createCarCmd(self.v * 1.2, self.omega))
            if self.leds != self.prev_leds:
                self.set_LEDs(self.leds)
                self.prev_leds = self.leds
            rate.sleep()

    def processImage(self, image_msg):
        #rospy.loginfo("Start processing image")

        start_time = time.time()
        try:
            image_cv = self.bridge.compressed_imgmsg_to_cv2(image_msg, "bgr8")
        except ValueError as e:
            rospy.loginfo("image decode error", e)
            return
        
        # Resize and crop image
        hei_original, wid_original = image_cv.shape[0:2]

        if self.resolution[0] != hei_original or self.resolution[1] != wid_original:
            image_cv = cv.resize(image_cv, (self.resolution[1], self.resolution[0]), interpolation=cv.INTER_NEAREST)
        hsv = cv.cvtColor(image_cv, cv.COLOR_BGR2HSV)
        mask = cv.inRange(hsv, self.green_low, self.green_high)

        contours = cv.findContours(mask.copy(), cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)[-2]

        if len(contours)>1:
            duckie_area = max(contours, key=cv.contourArea)
            (x,y,w,h) = cv.boundingRect(duckie_area)
            area = w * h
            x_avg = (2 * x + w) / 2
            rospy.loginfo(f"X: {x_avg}, Y: {(2 * y + h) / 2}, AREA: {area}")
            self.unseen_count = 0
            self.leds = ["white", "blue", "white", "blue", "white"]
            self.omega = max(min((90 - x_avg) / 45, 2), -2) # angle is calculated based off average x-position
            if area > 400: # 1000 would be good for no delay
                self.omega = 0
                self.v = 0
                self.leds = ["white", "purple", "white", "purple", "white"]
                rospy.loginfo(f"STOPPING: {self.omega}")
            elif area > 75: # 400 would be good for no delay
                self.v = 0.15
                rospy.loginfo(f"SLOWING, CLOSE: {self.omega}")
                self.leds = ["white", "pink", "white", "pink", "white"]
            elif area > 10: # 75 would be good for no delay
                self.v = 0.2
                rospy.loginfo(f"SLOW: {self.omega}")
                self.leds = ["white", "red", "white", "red", "white"]
            else:
                self.v = 0.25
                rospy.loginfo(f"FAR AWAY: {self.omega}")
        elif self.unseen_count > 10:
            self.unseen_count = 0
            self.leds = ["white", "white", "white", "white", "white"]
            # look around for other duckiebot
            if self.spin_count > 5 or (self.drive_count > 0 and self.drive_count < 5):
                self.omega = 0
                self.v = 0.2
                self.spin_count = 0
                self.drive_count += 1
                rospy.loginfo(f"RUNNING FORWARD: {self.drive_count}")
            else:
                # no objects detected - spin and look for objects?
                self.omega = 2
                self.v = 0.2
                self.spin_count += 1
                self.drive_count = 0
                rospy.loginfo(f"SPINNING: {self.spin_count}")
        else:
            self.unseen_count += 1

    
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

        #super(DaughterControlNode, self).onShutdown()

if __name__ == '__main__':
    # create the node
    node = DaughterControlNode(node_name='daughter_control_node')
    # run node
    node.run()
    node.onShutdown()
    # keep spinning
    rospy.spin()
