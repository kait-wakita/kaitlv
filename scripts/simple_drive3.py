#!/usr/bin/env python

from os import TMP_MAX
import rospy,copy,math,time
from geometry_msgs.msg import Twist
from std_srvs.srv import Trigger, TriggerResponse
from sensor_msgs.msg import LaserScan

class lidar():
    def __init__(self):
        self.cmd_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.scan_init_p = 0

        self.dist_r = 0
        self.dist_c = 0
        self.dist_l = 0
        
        self.danger_front = 0.7
        self.danger_side = 0.5
        rospy.Subscriber('scan', LaserScan, self.urg_callback)

    def urg_callback(self,m):
        if self.scan_init_p == 0:
            self.angle_min = m.angle_min
            self.angle_max = m.angle_max
            self.angle_inc = m.angle_increment

            self.scan_len = int(round((self.angle_max-self.angle_min) / self.angle_inc))
            i_center = int(self.scan_len/2)
            i_diff_center = int(round(10*math.pi/180/self.angle_inc))
            i_diff_side = int(round(30*math.pi/180/self.angle_inc))

            self.range_values = [5.0] * self.scan_len
            self.i_r1 = i_center - i_diff_center
            self.i_r2 = i_center - i_diff_side
            self.i_l1 = i_center + i_diff_center
            self.i_l2 = i_center + i_diff_side

            self.scan_init_p =1

        r_tmp = []
        for val in m.ranges:
            r_tmp.append(self.range_fix(val))
        self.dist_r = min(r_tmp[self.i_r2:self.i_r1])
        self.dist_c = min(r_tmp[self.i_r1:self.i_l1])
        self.dist_l = min(r_tmp[self.i_l1:self.i_l2])
        self.range_values = r_tmp


    def range_fix(self,r):
        if r<0.1:
            return(5.0)
        elif math.isnan(r):
            return(5.0)
        else:
            return(r)

    def run(self):
        rate =rospy.Rate(10)
        data = Twist()

        while not rospy.is_shutdown():
            data.linear.x = 1.0
            data.angular.z = 0.0

            if(self.dist_c < self.danger_front):
                if(self.dist_r > self.dist_l):
                    data.angular.z = -math.pi
                else:
                    data.angular.z = math.pi
            elif(self.dist_l < self.danger_side):
                data.angular.z = -math.pi/4
            elif(self.dist_r < self.danger_side):
                data.angular.z = math.pi/4
            else:
                data.linear.x = 1.5
         
            print("v=%4.2f w=%4.2f (left:%4.2f center:%4.2f right:%4.2f)" % (data.linear.x, data.angular.z, self.dist_l,self.dist_c,self.dist_r))
        
            self.cmd_vel.publish(data)
            rate.sleep()
            


if __name__ == '__main__':
    rospy.init_node('monitor_scan')
    lidar().run()

