import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import PoseStamped, Point
from builtin_interfaces.msg import Duration
import numpy as np

class MultiRobotExplorer(Node):
    def __init__(self):
        super().__init__('multi_robot_explorer')

        self.declare_parameter('min_unknown_cells', 15)
        self.min_unknown_cells = self.get_parameter('min_unknown_cells').get_parameter_value().integer_value

        self.map_sub = self.create_subscription(OccupancyGrid, '/map', self.map_callback, 10)
        self.marker_pub = self.create_publisher(MarkerArray, '/global_frontiers', 10)

        self.global_map = None

    def map_callback(self, msg):
        self.global_map = msg
        frontiers = self.find_frontiers(msg)
        self.publish_frontier_markers(frontiers, msg)

    def find_frontiers(self, map_msg):
        height = map_msg.info.height
        width = map_msg.info.width
        data = np.array(map_msg.data, dtype=np.int8).reshape((height, width))
        frontiers = []

        for y in range(2, height - 2):
            for x in range(2, width - 2):
                if data[y, x] != 0:
                    continue
                neighborhood = data[y-1:y+2, x-1:x+2].flatten()
                if -1 not in neighborhood:
                    continue
                unknown_area = data[y-2:y+3, x-2:x+3]
                if np.sum(unknown_area == -1) < self.min_unknown_cells:
                    continue
                if np.sum(unknown_area == 100) > 2:
                    continue
                frontiers.append((y, x))

        return frontiers

    def publish_frontier_markers(self, frontiers, map_msg):
        marker_array = MarkerArray()
        marker = Marker()
        marker.header.frame_id = "world"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "global_frontiers"
        marker.id = 0
        marker.type = Marker.SPHERE_LIST
        marker.action = Marker.ADD
        marker.scale.x = 0.2
        marker.scale.y = 0.2
        marker.scale.z = 0.2
        marker.color.r = 1.0
        marker.color.g = 0.5
        marker.color.b = 0.0
        marker.color.a = 1.0
        marker.lifetime = Duration(sec=2)

        resolution = map_msg.info.resolution
        origin = map_msg.info.origin.position

        for y, x in frontiers:
            px = origin.x + (x + 0.5) * resolution
            py = origin.y + (y + 0.5) * resolution
            marker.points.append(Point(x=px, y=py, z=0.1))

        marker_array.markers.append(marker)
        self.marker_pub.publish(marker_array)


def main(args=None):
    rclpy.init(args=args)
    node = MultiRobotExplorer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
