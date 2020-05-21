#!/usr/bin/env python

import rospy as rp
from std_msgs.msg import Bool
from sensor_msgs.msg import LaserScan

import numpy as np

COLLISION_DISTANCE = 0.5


class Laser:
    """
    A class used to handle LaserScan messages from /scan topic and publish /collision topic.

    ...

    Attributes
    ----------
    laser_ranges : numpy.ndarray
        Array with LaserScan measuremnts limited to +-90 degrees.

    collision_pub : rospy.Publisher
        Bool messages publisher of collision information.

    Methods
    -------
    laser_callback(data)
        Get LaserScan data from subscribed /scan topic and limit it to +-90 degrees.
        
    is_collision()
        Check if any of laser_ranges data isn't closer than COLLISION_DISTANCE and publish proper collision message.
    """

    def __init__(self):
        self.laser_ranges = None

        rp.init_node('collision_detector')
        rp.Subscriber('/scan', LaserScan, self.laser_callback)

        self.collision_pub = rp.Publisher('collision', Bool, queue_size=5)

    def laser_callback(self, data):
        """Get LaserScan data from subscribed /scan topic and limit it to +-90 degrees.

        Parameters
        ----------
        data : list
            LaserScan data from subscribed /scan topic.
        """

        scan_range = int((data.angle_max - np.pi/2) / data.angle_increment)
        self.laser_ranges = np.array(data.ranges[scan_range:(-scan_range)])
        self.is_collision()

    def is_collision(self):
        """Check if any of laser_ranges data isn't closer than COLLISION_DISTANCE and publish proper collision message."""

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
