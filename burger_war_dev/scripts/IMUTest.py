#!/usr/bin/env python
# -*- coding: utf-8 -*-
'''
This is rumdom run node.
subscribe No topcs.
Publish 'cmd_vel' topic. 
mainly use for simple sample program

by Takuya Yamaguhi.
'''

import rospy
import random
import tf

from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Vector3

class RandomBot():
    def __init__(self, bot_name="NoName"):
        # bot name 
        self.name = bot_name
        # velocity publisher
        self.vel_pub = rospy.Publisher('cmd_vel', Twist,queue_size=1)
        self.imu = Imu()
        self.imu_sub = rospy.Subscriber('imu', Imu, self.imuCallback)
        self.done = 0
        

    def imuCallback(self, data):
                self.imu = data

    def calcTwist(self):
        """
        value = random.randint(1,1000)
        if value < 250:
            x = 0.2
            th = 0
        elif value < 500:
            x = -0.2
            th = 0
        elif value < 750:
            x = 0
            th = 1
        elif value < 1000:
            x = 0
            th = -1
        else:
            x = 0
            th = 0
        """
        if self.done == 0:
            twist = Twist()
            twist.linear.x = 0; twist.linear.y = 0; twist.linear.z = 0
            twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = 1
            self.done = 1
            return twist

    def q_to_e(self):
        quaternion = self.imu.orientation
        e = tf.transformations.euler_from_quaternion((quaternion.x,quaternion.y,quaternion.z,quaternion.w))
        return Vector3(x=e[0],y=e[1],z=e[2]-(3.14/2))

    def strategy(self):
        r = rospy.Rate(1) # change speed 1fps
        count = 0
        target_speed = 0
        target_turn = 0
        control_speed = 0
        control_turn = 0
        twist = Twist()
        while not rospy.is_shutdown():
            twist.linear.x = 0; twist.linear.y = 0; twist.linear.z = 0
            twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = 0 
            self.vel_pub.publish(twist)
            if len(self.imu.orientation_covariance) != 0:
                #print self.imu
                quaternion = self.imu.orientation
                #print quaternion
                euler = bot.q_to_e()
                print euler
            r.sleep()

"""
        while not rospy.is_shutdown():
            twist = self.calcTwist()
            print(twist)
            self.vel_pub.publish(twist)

            r.sleep()
"""

if __name__ == '__main__':
    rospy.init_node('random_run')
    bot = RandomBot('Random')
    bot.strategy()

