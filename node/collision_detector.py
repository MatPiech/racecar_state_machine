#!/usr/bin/env python

import rospy as rp
from std_msgs.msg import Bool
from sensor_msgs.msg import LaserScan

import numpy as np

PI = 3.14
COLLISION_DISTANCE = 0.5


class Laser:
    def __init__(self):
        self.laser_ranges = None

        rp.init_node('collision_detector')
        rp.Subscriber('/scan', LaserScan, self.laser_callback)

        self.collision_pub = rp.Publisher('collision', Bool, queue_size=5)

    def laser_callback(self, data):
        scan_range = int((data.angle_max - PI/2) / data.angle_increment)
        self.laser_ranges = np.array(data.ranges[scan_range:(-scan_range)])
        self.is_collision()

    def is_collision(self):
        collision_msg = Bool()

        if (self.laser_ranges < COLLISION_DISTANCE).any():
            collision_msg.data = True
        else:
            collision_msg.data = False
        
        self.collision_pub.publish(collision_msg)
        


if __name__ == "__main__":
    laser = Laser()
    while not rp.is_shutdown():
        rp.sleep(0.5)
