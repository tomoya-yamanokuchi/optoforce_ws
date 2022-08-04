import time
import numpy as np
import rospy
from geometry_msgs.msg import Vector3


class Optoforce_Subscriber:
    def __init__(self):
        rospy.Subscriber("/optoforce_node/optoforce_3D", Vector3, self._callback_)
        self.force                    = None
        self.connection_flag          = {}
        self.connection_flag["force"] = False
        self.subscribe_sum            = len(self.connection_flag)
        self._wait_connection()


    def _wait_connection(self):
        while not self._is_connection_established():
            time.sleep(0.3)
        rospy.loginfo("Optoforce_Subscriber is established!")


    def _is_connection_established(self):
        return np.sum(np.array(list(self.connection_flag.values()))*1) == self.subscribe_sum


    def _callback_(self, data):
        self.force = np.array([data.x, data.y, data.z])
        self.connection_flag["force"] = True


if __name__ == '__main__':

    rospy.init_node("optforce_node")
    sub = Optoforce_Subscriber()

    for i in range(1000):
        print("[{: .2f}, {: .2f}, {: .2f}]".format(*sub.force))
        time.sleep(0.1)
