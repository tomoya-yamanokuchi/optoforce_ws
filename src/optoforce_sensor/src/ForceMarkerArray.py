import rospy
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point, Quaternion, Vector3
from std_msgs.msg import ColorRGBA
from RGBA import RGBA


class ForceMarkerArray:
    def __init__(self, namespace="ft1", arrow_position_offset=[0.0, 0.0, 0.0]):
        self.frame_id              = "world"
        self.namespace             = namespace
        self.n_axis                = 3 # MAF-3の軸数（Fz, Mx, My）
        self.arrow_scale           = Vector3(x=0.1, y=0.3, z=0.5) # (arrow_length, arrow_width, arrow_height)
        self.orientation           = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)

        self.arrow_position_offset = arrow_position_offset
        self.position              = Point(
            x = self.arrow_position_offset[0],
            y = self.arrow_position_offset[1],
            z = self.arrow_position_offset[2]
        )


    def create_marker(self, id, xyz, rgba):
        marker                  = Marker()
        marker.header.frame_id  = self.frame_id
        marker.header.stamp     = rospy.Time.now()
        marker.ns               = self.namespace
        marker.id               = id
        marker.lifetime         = rospy.Duration()
        marker.type             = Marker.ARROW
        marker.action           = Marker.ADD
        marker.color            = ColorRGBA(r=rgba[0], g=rgba[1], b=rgba[2], a=rgba[3])
        marker.scale            = self.arrow_scale
        marker.pose.position    = self.position
        marker.pose.orientation = self.orientation
        marker.points.append(self.position)
        marker.points.append(Point(
            x = self.arrow_position_offset[0] + xyz[0],
            y = self.arrow_position_offset[1] + xyz[1],
            z = self.arrow_position_offset[2] + xyz[2]
        ))
        return marker


    def create(self, weight, marker_array=None):
        assert len(weight) == self.n_axis
        if marker_array is None:
            marker_array = MarkerArray()
        marker_array.markers.append(self.create_marker(id=0, xyz=[ 0.0, 0.0, -weight[2]], rgba=RGBA([255,  64, 129, 0.3]).rgba_max1())) # Fz
        marker_array.markers.append(self.create_marker(id=1, xyz=[ 0.0,  weight[1], 0.0], rgba=RGBA([118, 255,   3, 0.3]).rgba_max1())) # Fx
        marker_array.markers.append(self.create_marker(id=2, xyz=[  weight[0], 0.0, 0.0], rgba=RGBA([  0, 176, 255, 0.3]).rgba_max1())) # Fy

        marker_array.markers.append(self.create_marker(id=3, xyz=[weight[0], weight[1], -weight[2]], rgba=RGBA([ 255, 0, 255, 1.0]).rgba_max1())) # (Fx, Fy, Fz)
        return marker_array

