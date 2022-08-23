#!/usr/bin/env python

from os import TMP_MAX
import rospy,copy,math,time
from geometry_msgs import Twist
from std_srvs.srv import Trigger, TriggerResponse
from sensor_msgs.msg import LaserScan

class lidar():
    def __init__(self):
        self.cmd_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.range_values = [5.0] * 726
        self.danger_front = 0.5
        self.danger_diagonal = 0.4
        self.danger_side = 0.4
        rospy.Subscriber('scan', LaserScan, self.urg_callback)

    def urg_callback(self,m):
        r_tmp = [];
        for val in m.ranges:
            r_tmp.append(self.range_fix(val))
        self.range_values = r_tmp

    def range_fix(self,r):
        if r<0.1:
            return(5.0)
        elif math.isnan(r):
            return(5.0)
        else:
            return(r)

    def too_right(self):
        return(min(self.range_values[240:355])<self.danger_side)

    def wall_front(self):
        return(min(self.range_values[356:414])<self.danger_side)

    def too_left(self):
        return(min(self.range_values[415:530])<self.danger_side)

    def run(self):
        rate =rospy.Rate(10)
        data = Twist()

        while not rospy.is_shutdown():
            data.linear.x = 0.5
            data.angular.z = 0.0

            if self.wall_front():
                data.angular.z = 2.0*math.pi
            elif self.too_right():
                data.angular.z = math.pi/2
            elif self.too.left():
                data.angular.z = -math.pi/2
            
            print(data.linear.x, data.angular.z)
            self.cmd_vel_publisher(data)
            rate.sleep()
            


    

