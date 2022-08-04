from copy import copy
from email import header
from mimetypes import init
from black import main
from matplotlib.pyplot import axis, pink
from numpy import piecewise
import rospy
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point, Quaternion, Vector3
from std_msgs.msg import ColorRGBA

from Optoforce_Subscriber import Optoforce_Subscriber
from ForceMarkerArray import ForceMarkerArray

import numpy as np
import copy

class Optoforce_RVIZ:
    def __init__(self):
        self.a = 0


    def main(self):
        rospy.init_node("marker_array_node")
        self.opt_sub     = Optoforce_Subscriber()
        ft_marker_array  = ForceMarkerArray()
        marker_array_pub = rospy.Publisher("marker_array", MarkerArray, queue_size = 100)

        rate = rospy.Rate(20)

        num_init = 100
        force_init = np.zeros([num_init, 3])
        for i in range(num_init):
            force_init[i] = copy.deepcopy(self.opt_sub.force)
            rate.sleep()
        force_init_mean = np.mean(force_init, axis=0)
        print("force_init_mean : ", force_init_mean)

        while not rospy.is_shutdown():
            force = (self.opt_sub.force - force_init_mean) * 0.01
            marker_array_msg = ft_marker_array.create(force)
            marker_array_pub.publish(marker_array_msg)

            print("[{: .2f}, {: .2f}, {: .2f}]".format(*force))
            rate.sleep()


if __name__ == '__main__':
    rviz = Optoforce_RVIZ()

    try:
        rviz.main()
    finally:
        print("close!")

