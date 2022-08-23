#!/usr/bin/env python

from os import TMP_MAX
import rospy,copy,math,time
from geometry_msgs.msg import Twist
from std_srvs.srv import Trigger, TriggerResponse
from sensor_msgs.msg import LaserScan

class lidar():
    def __init__(self):
        self.cmd_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.angle_min = -1.57
        self.angle_max = 1.57
        self.angle_inc = 0.00314

        self.scan_len = int(round((self.angle_max-self.angle_min) / self.angle_inc))
        i_center = int(self.scan_len/2)
        i_diff_center = int(round(10*math.pi/180/self.angle_inc))
        i_diff_side = int(round(20*math.pi/180/self.angle_inc))

        self.range_values = [5.0] * self.scan_len
        self.i_r1 = i_center - i_diff_center
        self.i_r2 = i_center - i_diff_side
        self.i_l1 = i_center + i_diff_center
        self.i_l2 = i_center + i_diff_side

        self.dist_r = 0
        self.dist_c = 0
        self.dist_l = 0
        
        self.danger_front = 0.5
        self.danger_diagonal = 0.4
        self.danger_side = 0.4
        rospy.Subscriber('scan', LaserScan, self.urg_callback)

    def urg_callback(self,m):
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
        rate =rospy.Rate(1)
        data = Twist()

        while not rospy.is_shutdown():
            data.linear.x = 0.5
            data.angular.z = 0.0
            
            print("left:%4.2f center:%4.2f right:%4.2f" % (self.dist_l,self.dist_c,self.dist_r))
            #print(data.linear.x, data.angular.z)

            #self.cmd_vel.publish(data)
            rate.sleep()
            


if __name__ == '__main__':
    rospy.init_node('monitor_scan')
    lidar().run()

