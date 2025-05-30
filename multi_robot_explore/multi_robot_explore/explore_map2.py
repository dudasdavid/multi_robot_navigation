import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import SetParametersResult
from nav_msgs.msg import OccupancyGrid
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import PoseStamped, Point
from builtin_interfaces.msg import Duration
from tf2_geometry_msgs.tf2_geometry_msgs import do_transform_pose
#from tf2_ros import Buffer, TransformListener
from bitbots_tf_buffer import Buffer


import numpy as np

class MultiRobotExplorer(Node):
    def __init__(self):
        super().__init__('multi_robot_explorer')

        self.declare_parameter('min_unknown_cells', 12)
        self.min_unknown_cells = self.get_parameter('min_unknown_cells').get_parameter_value().integer_value
        
        # TODO: has to be fixed, it forces use_sim_time to True
        self.set_parameters([rclpy.parameter.Parameter('use_sim_time', rclpy.Parameter.Type.BOOL, True)])
        self.use_sim_time = self.get_parameter('use_sim_time').get_parameter_value().bool_value

        self.add_on_set_parameters_callback(self.update_parameter_callback)

        self.map_sub = self.create_subscription(OccupancyGrid, '/map', self.map_callback, 10)
        self.marker_pub = self.create_publisher(MarkerArray, '/global_frontiers', 10)
        self.robot_frames = {
            'robot_1': 'robot_1/base_link',
            'robot_2': 'robot_2/base_link'
        }
        
        self.goal_pubs = {
            'robot_1': self.create_publisher(PoseStamped, '/robot_1/goal_pose', 10),
            'robot_2': self.create_publisher(PoseStamped, '/robot_2/goal_pose', 10)
        }

        self.global_map = None
        
        self.tf_buffer = Buffer(self)
        #self.tf_listener = TransformListener(self.tf_buffer, self)

        self.current_targets = {
            'robot_1': None,
            'robot_2': None
        }
        self.target_start_times = {
            'robot_1': None,
            'robot_2': None
        }

        self.blacklists = {
            'robot_1': set(),
            'robot_2': set()
        }

    def update_parameter_callback(self, params):
        result = SetParametersResult(successful=True)
        for param in params:
            if param.name == 'min_unknown_cells' and param.type_ == rclpy.Parameter.Type.INTEGER:
                self.min_unknown_cells = param.value
                self.get_logger().info(f'Updating minimum unknown cells threshold to {self.min_unknown_cells}')
                return result
        return result

    def get_home_pose(self, map_msg, robot_name):
        try:
            # 0,0 in robot_x/map
            map_frame = f"{robot_name}/map"
            pose_in_map = PoseStamped()
            pose_in_map.header.frame_id = map_frame
            pose_in_map.header.stamp = self.get_clock().now().to_msg()
            pose_in_map.pose.position.x = 0.0
            pose_in_map.pose.position.y = 0.0
            pose_in_map.pose.position.z = 0.0
            pose_in_map.pose.orientation.w = 1.0

            transform = self.tf_buffer.lookup_transform(
                'world', map_frame, rclpy.time.Time(), timeout=rclpy.duration.Duration(seconds=0.2)
            )
            pose_world = do_transform_pose(pose_in_map.pose, transform)

            # Convert world pose to global map cell
            resolution = map_msg.info.resolution
            origin = map_msg.info.origin.position
            x = int((pose_world.position.x - origin.x) / resolution)
            y = int((pose_world.position.y - origin.y) / resolution)
            return (y, x)

        except Exception as e:
            self.get_logger().warn(f"Failed to compute home cell for {robot_name}: {e}")
            return None

    def get_closest_frontier(self, frontiers, map_msg, robot_frame):
        resolution = map_msg.info.resolution
        origin = map_msg.info.origin.position
        try:
            transform = self.tf_buffer.lookup_transform(
                map_msg.header.frame_id, robot_frame, rclpy.time.Time(), timeout=rclpy.duration.Duration(seconds=0.1)
            )
            rx = transform.transform.translation.x
            ry = transform.transform.translation.y
            def frontier_distance(f):
                y, x = f
                fx = origin.x + (x + 0.5) * resolution
                fy = origin.y + (y + 0.5) * resolution
                return np.hypot(fx - rx, fy - ry)
            return min(frontiers, key=frontier_distance)
        except Exception as e:
            self.get_logger().warn(f"TF transform failed for {robot_frame}: {e}")
            return None

    def transform_blacklists_to_world(self, map_msg):
        resolution = map_msg.info.resolution
        origin = map_msg.info.origin.position
        world_points = []

        for robot, blacklist in self.blacklists.items():
            frame = f"{robot}/map"
            try:
                transform = self.tf_buffer.lookup_transform(
                    'world', frame, rclpy.time.Time(), timeout=rclpy.duration.Duration(seconds=0.1))
                for y, x in blacklist:
                    local_x = origin.x + (x + 0.5) * resolution
                    local_y = origin.y + (y + 0.5) * resolution
                    pose = PoseStamped()
                    pose.header.frame_id = frame
                    pose.pose.position.x = local_x
                    pose.pose.position.y = local_y
                    pose.pose.position.z = 0.1
                    pose.pose.orientation.w = 1.0
                    try:
                        transformed = do_transform_pose(pose.pose, transform)
                        world_points.append(Point(
                            x=transformed.position.x,
                            y=transformed.position.y,
                            z=transformed.position.z))
                    except Exception as e:
                        self.get_logger().warn(f"Transform failed for blacklist {robot} cell {(y,x)}: {e}")
            except Exception as e:
                self.get_logger().warn(f"TF lookup failed for {frame} to world: {e}")

        return world_points

    def publish_blacklist_markers(self, world_points):
        marker_array = MarkerArray()
        marker = Marker()
        marker.header.frame_id = "world"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "blacklisted_frontiers"
        marker.id = 0
        marker.type = Marker.SPHERE_LIST
        marker.action = Marker.ADD
        marker.scale.x = 0.2
        marker.scale.y = 0.2
        marker.scale.z = 0.2
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 1.0
        marker.lifetime = Duration(sec=2)

        marker.points.extend(world_points)
        marker_array.markers.append(marker)
        self.marker_pub.publish(marker_array)

    def map_callback(self, msg):

        self.check_and_blacklist_stuck_targets(msg)
        self.global_map = msg
        frontiers = self.find_frontiers(msg)
        # filter out blacklisted frontiers in world frame
        blacklist_points = self.transform_blacklists_to_world(msg)
        self.publish_blacklist_markers(blacklist_points)

        filtered_frontiers = []
        for cell in frontiers:
            py = msg.info.origin.position.y + (cell[0] + 0.5) * msg.info.resolution
            px = msg.info.origin.position.x + (cell[1] + 0.5) * msg.info.resolution
            if all(np.hypot(px - p.x, py - p.y) > 0.8 for p in blacklist_points):
                filtered_frontiers.append(cell)

        self.publish_frontier_markers(filtered_frontiers, msg)
        frontiers = filtered_frontiers

        if len(frontiers) == 0:
            self.get_logger().info("No more frontiers found")

            for robot, frame in self.robot_frames.items():
                home = self.get_home_pose(msg, robot)
                self.publish_goal_pose(home, msg, robot)

        else:
            self.get_logger().info(f"Found {len(frontiers)} frontiers")

            for robot, frame in self.robot_frames.items():
                closest = self.get_closest_frontier(frontiers, msg, frame)
                if closest:
                    self.publish_selected_frontier(closest, msg, robot)
                    self.publish_goal_pose(closest, msg, robot)

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

    def publish_selected_frontier(self, cell, map_msg, robot_name):
        marker_array = MarkerArray()
        marker = Marker()
        marker.header.frame_id = "world"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = f"{robot_name}_goal"
        marker.id = 0
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.scale.x = 0.4
        marker.scale.y = 0.4
        marker.scale.z = 0.4
        if robot_name == 'robot_2':
            marker.color.r = 0.0
            marker.color.g = 1.0
            marker.color.b = 0.0
        else:
            marker.color.r = 0.0
            marker.color.g = 0.0
            marker.color.b = 1.0
        marker.color.a = 1.0
        marker.lifetime = Duration(sec=2)

        resolution = map_msg.info.resolution
        origin = map_msg.info.origin.position
        y, x = cell
        px = origin.x + (x + 0.5) * resolution
        py = origin.y + (y + 0.5) * resolution
        marker.pose.position.x = px
        marker.pose.position.y = py
        marker.pose.position.z = 0.1
        marker.pose.orientation.w = 1.0

        marker_array.markers.append(marker)
        self.marker_pub.publish(marker_array)

    def check_and_blacklist_stuck_targets(self, map_msg):
        now = self.get_clock().now().nanoseconds / 1e9
        resolution = map_msg.info.resolution
        origin = map_msg.info.origin.position

        for robot, target in self.current_targets.items():
            if not target:
                continue
            start_time = self.target_start_times[robot]
            if not start_time or now - start_time < 10.0:
                #self.get_logger().info(f"{robot} is not stuck: {now - start_time} seconds since last target")
                continue

            try:
                self.get_logger().info(f"{robot} is stuck for {now - start_time} seconds, checking reachability")
                transform = self.tf_buffer.lookup_transform(
                    'world', f'{robot}/base_link', rclpy.time.Time(), timeout=rclpy.duration.Duration(seconds=0.1)
                )
                rx = transform.transform.translation.x
                ry = transform.transform.translation.y

                y, x = target
                tx = origin.x + (x + 0.5) * resolution
                ty = origin.y + (y + 0.5) * resolution

                if np.hypot(tx - rx, ty - ry) < 1.0:
                    self.get_logger().warn(f"Blacklisting unreachable frontier for {robot} at ({y}, {x})")

                    try:
                        transform_to_local = self.tf_buffer.lookup_transform(
                            f'{robot}/map', 'world', rclpy.time.Time(), timeout=rclpy.duration.Duration(seconds=0.1)
                        )
                        pose_world = PoseStamped()
                        pose_world.header.frame_id = 'world'
                        pose_world.pose.position.x = tx
                        pose_world.pose.position.y = ty
                        pose_world.pose.position.z = 0.0
                        pose_world.pose.orientation.w = 1.0

                        transformed_pose = do_transform_pose(pose_world.pose, transform_to_local)

                        lx = int((transformed_pose.position.x - origin.x) / resolution)
                        ly = int((transformed_pose.position.y - origin.y) / resolution)

                        self.blacklists[robot].add((ly, lx))
                    except Exception as e:
                        self.get_logger().warn(f"Failed to transform stuck frontier to {robot}/map: {e}")
                    self.current_targets[robot] = None
                    self.target_start_times[robot] = None

            except Exception as e:
                self.get_logger().warn(f"TF transform failed for stuck check {robot}: {e}")

    def publish_goal_pose(self, cell, map_msg, robot_name):
        pub = self.goal_pubs[robot_name]
        pose = PoseStamped()
        pose.header.frame_id = "world"
        pose.header.stamp = self.get_clock().now().to_msg()

        resolution = map_msg.info.resolution
        origin = map_msg.info.origin.position
        y, x = cell
        pose.pose.position.x = origin.x + (x + 0.5) * resolution
        pose.pose.position.y = origin.y + (y + 0.5) * resolution
        pose.pose.position.z = 0.0
        pose.pose.orientation.w = 1.0

        pub.publish(pose)
        if self.current_targets[robot_name] != cell:
            self.current_targets[robot_name] = cell
            self.target_start_times[robot_name] = self.get_clock().now().nanoseconds / 1e9
        self.get_logger().info(f"Sent goal for {robot_name} to ({pose.pose.position.x:.2f}, {pose.pose.position.y:.2f})")

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
