import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import OccupancyGrid
from rosgraph_msgs.msg import Clock
from rcl_interfaces.msg import SetParametersResult
from std_msgs.msg import Header
import tf2_ros
import numpy as np
import cv2
import math
import threading
import tf_transformations
import time
from shapely.geometry import Polygon, Point

class MultiRobotMapMerger(Node):
    def __init__(self):
        super().__init__('multi_robot_map_merger')

        self.declare_parameter('tf_publish_frequency', 20.0)
        self.declare_parameter('map_publish_frequency', 1.0)
        self.declare_parameter('visualize', True)
        self.declare_parameter('match_confidence_threshold', 0.6)
        self.declare_parameter('min_map_size', 60000)

        self.publish_frequency = self.get_parameter('tf_publish_frequency').get_parameter_value().double_value
        self.map_publish_frequency = self.get_parameter('map_publish_frequency').get_parameter_value().double_value
        self.visualize = self.get_parameter('visualize').get_parameter_value().bool_value
        self.confidence_threshold = self.get_parameter('match_confidence_threshold').get_parameter_value().double_value
        self.min_map_size = self.get_parameter('min_map_size').get_parameter_value().integer_value
        self.use_sim_time = self.get_parameter('use_sim_time').get_parameter_value().bool_value

        self.add_on_set_parameters_callback(self.update_parameter_callback)

        self.robot1_pos = (0.0, 0.0)
        self.robot2_pos = (0.0, 0.0)

        self.broadcaster = tf2_ros.TransformBroadcaster(self)
        self.map_publisher = self.create_publisher(OccupancyGrid, '/map', 10)

        self.timer = self.create_timer(1.0 / self.publish_frequency, self.timer_callback)
        self.map_timer = self.create_timer(1.0 / self.map_publish_frequency, self.map_publish_callback)

        self.map1_img = None
        self.map2_img = None
        self.map1_info = None
        self.map2_info = None
        self.merged_map_img = None
        self.proc1_img = None
        self.proc2_img = None
        self.warped = None

        self.create_subscription(OccupancyGrid, '/robot_1/map', self.map1_callback, 10)
        self.create_subscription(OccupancyGrid, '/robot_2/map', self.map2_callback, 10)

        if self.visualize:
            self.vis_thread = threading.Thread(target=self.visualization_loop, daemon=True)
            self.vis_thread.start()

    def update_parameter_callback(self, params):
        result = SetParametersResult(successful=True)
        for param in params:
            if param.name == 'match_confidence_threshold' and param.type_ == rclpy.Parameter.Type.DOUBLE:
                self.confidence_threshold = param.value
                self.get_logger().info(f'Updating minimum confidence threshold to {self.confidence_threshold}')
                return result
            elif param.name == 'min_map_size' and param.type_ == rclpy.Parameter.Type.INTEGER:
                self.min_map_size = param.value
                self.get_logger().info(f'Updating minimum map size to {self.min_map_size}')
                return result
        # Return success, so updates are seen via get_parameter()
        return result

    def occupancy_grid_to_image(self, msg):
        width = msg.info.width
        height = msg.info.height
        data = np.array(msg.data, dtype=np.int8).reshape((height, width))
        img = np.zeros((height, width), dtype=np.uint8)
        img[data == -1] = 127 # Unknown cells
        img[data == 0] = 255  # Free cells
        img[data > 0] = 0     # Occupied cells
        return img

    def preprocess_image(self, img):
        # invert early
        img = cv2.bitwise_not(img)
        # CLAHE to boost line contrast
        clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))
        enhanced = clahe.apply(img)

        # Light blur to suppress pixel noise, preserve edges
        blurred = cv2.GaussianBlur(enhanced, (3, 3), sigmaX=0.8)

        # Inflate the walls
        kernel = np.ones((3, 3), np.uint8)
        inflated = cv2.dilate(blurred, kernel, iterations=1)

        return inflated

    def check_map_overlap_orb(self, img1, img2):
        orb = cv2.ORB_create(1000)
        kp1, des1 = orb.detectAndCompute(img1, None)
        kp2, des2 = orb.detectAndCompute(img2, None)

        if des1 is None or des2 is None:
            return 0.0, None, None, None

        bf = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)
        matches = bf.match(des1, des2)
        if not matches:
            return 0.0, None, None, None

        matches = sorted(matches, key=lambda x: x.distance)
        good_matches = [m for m in matches if m.distance < 60]
        confidence = 0.0  # placeholder for now
        return confidence, kp1, kp2, good_matches

    def map1_callback(self, msg):
        self.map1_img = self.occupancy_grid_to_image(msg)
        self.map1_info = msg.info

    def map2_callback(self, msg):
        self.map2_img = self.occupancy_grid_to_image(msg)
        self.map2_info = msg.info

    def timer_callback(self):

        now = self.get_clock().now().to_msg()

        # Robot 1 TF
        tf1 = TransformStamped()
        tf1.header.stamp = now
        tf1.header.frame_id = 'world'
        tf1.child_frame_id = 'robot_1/map'
        tf1.transform.translation.x = self.robot1_pos[0]
        tf1.transform.translation.y = self.robot1_pos[1]
        tf1.transform.translation.z = 0.0
        tf1.transform.rotation.x = 0.0
        tf1.transform.rotation.y = 0.0
        tf1.transform.rotation.z = 0.0
        tf1.transform.rotation.w = 1.0
        self.broadcaster.sendTransform(tf1)

        # Robot 2 TF with rotation
        tf2 = TransformStamped()
        tf2.header.stamp = now
        tf2.header.frame_id = 'world'
        tf2.child_frame_id = 'robot_2/map'
        tf2.transform.translation.x = self.robot2_pos[0]
        tf2.transform.translation.y = self.robot2_pos[1]
        tf2.transform.translation.z = 0.0
        q = tf_transformations.quaternion_from_euler(0, 0, getattr(self, 'robot2_yaw', 0.0))
        tf2.transform.rotation.x = q[0]
        tf2.transform.rotation.y = q[1]
        tf2.transform.rotation.z = q[2]
        tf2.transform.rotation.w = q[3]
        self.broadcaster.sendTransform(tf2)

    def map_publish_callback(self):
        if self.map1_img is None or self.map2_img is None:
            return

        img1 = self.preprocess_image(self.map1_img)
        img2 = self.preprocess_image(self.map2_img)
        _, kp1, kp2, good_matches = self.check_map_overlap_orb(img1, img2)

        self.proc1_img = img1.copy()
        self.proc2_img = img2.copy()

        if good_matches is not None:
            # Extract matching points
            src_pts = np.float32([kp2[m.trainIdx].pt for m in good_matches]).reshape(-1, 1, 2)
            dst_pts = np.float32([kp1[m.queryIdx].pt for m in good_matches]).reshape(-1, 1, 2)
            
            if src_pts.shape[0] < 1 or dst_pts.shape[0] < 1:
                confidence = 0.0
                self.get_logger().info("Not enough points for transformation.")

            else:
                # Estimate affine transform
                M, inliers = cv2.estimateAffinePartial2D(src_pts, dst_pts, method=cv2.RANSAC)
                if M is None or inliers is None:
                    confidence = 0.0
                    self.get_logger().info("Transform estimation failed.")
                
                else:
                    # Warp img2 corners into img1 frame to get estimated overlap polygon
                    h1, w1 = img1.shape[:2]
                    h2, w2 = img2.shape[:2]

                    map1_size = h1*w1
                    map2_size = h2*w2

                    corners2 = np.array([[0, 0], [w2, 0], [w2, h2], [0, h2]], dtype=np.float32).reshape(-1, 1, 2)
                    warped_corners = cv2.transform(corners2, M)  # 4x1x2
                    polygon = Polygon(warped_corners.reshape(-1, 2))
                    
                    # Count how many inlier points in kp1 lie inside the polygon
                    overlap_inlier_count = 0
                    for i, m in enumerate(good_matches):
                        if inliers[i]:
                            pt = kp1[m.queryIdx].pt
                            if polygon.contains(Point(pt)):
                                overlap_inlier_count += 1

                    # Count how many total keypoints from img1 fall in the polygon
                    kp1_pts = np.array([kp.pt for kp in kp1])
                    overlap_kp_count = sum(polygon.contains(Point(p)) for p in kp1_pts)

                    # Compute overlap-normalized confidence
                    if overlap_kp_count > 0:
                        confidence = overlap_inlier_count / overlap_kp_count
                    else:
                        confidence = 0.0

                    self.get_logger().info(f"Confidence: {confidence:.2f}, Inliers: {overlap_inlier_count}, Overlap keypoints: {overlap_kp_count}, Map 1 size: {img1.shape}, Map 2 size: {img2.shape}")
                    if map1_size < self.min_map_size or map2_size < self.min_map_size:
                        confidence = 0.0
                        self.get_logger().info("One of the maps is too small, confidence set to 0.0")
                        self.get_logger().info(f"Map 1 size: {map1_size}, Map 2 size: {map2_size}, Min size: {self.min_map_size}")
            
        else:
            confidence = 0.0
            self.get_logger().info("0 good matches found, confidence set to 0.0")

        if confidence < self.confidence_threshold:
            self.get_logger().info("Using fallback layout (maps not aligned)")
        else:
            self.get_logger().info("Merging maps based on feature alignment")

        res = self.map1_info.resolution

        if confidence < self.confidence_threshold:
            gap_pixels = int(1.0 / res)
            width = max(self.map1_img.shape[1], self.map2_img.shape[1])
            height = self.map1_img.shape[0] + gap_pixels + self.map2_img.shape[0]
            canvas = np.full((height, width), 127, dtype=np.uint8)
            canvas[0:self.map1_img.shape[0], 0:self.map1_img.shape[1]] = self.map1_img
            map2_offset_y = self.map1_img.shape[0] + gap_pixels
            canvas_region = canvas[map2_offset_y:map2_offset_y + self.map2_img.shape[0], 0:self.map2_img.shape[1]]
            mask = self.map2_img != 127
            canvas_region[mask] = self.map2_img[mask]
            self.robot1_pos = (-self.map1_info.origin.position.x, -self.map1_info.origin.position.y)
            self.robot2_pos = (-self.map2_info.origin.position.x, map2_offset_y * res - self.map2_info.origin.position.y)
            self.robot2_yaw = 0.0
        else:
            
            res = self.map1_info.resolution

            # Shapes
            h1, w1 = self.map1_img.shape
            h2, w2 = self.map2_img.shape

            # Corners of both maps
            map2_corners = np.float32([[0, 0], [w2, 0], [w2, h2], [0, h2]]).reshape(-1, 1, 2)
            map1_corners = np.float32([[0, 0], [w1, 0], [w1, h1], [0, h1]]).reshape(-1, 1, 2)

            # Transform map2 corners
            transformed_corners = cv2.transform(map2_corners, M)
            all_corners = np.vstack((map1_corners, transformed_corners))

            # Bounding box
            [x_min, y_min] = np.floor(np.min(all_corners, axis=0)[0]).astype(int)
            [x_max, y_max] = np.ceil(np.max(all_corners, axis=0)[0]).astype(int)
            canvas_w = x_max - x_min
            canvas_h = y_max - y_min

            # Offset transform to fit everything on positive canvas
            offset = np.array([[1, 0, -x_min],
                            [0, 1, -y_min]], dtype=np.float32)
            M_offset = offset @ np.vstack([M, [0, 0, 1]])
            M_offset = M_offset[:2]

            # Warp map2
            warped = cv2.warpAffine(self.map2_img, M_offset, (canvas_w, canvas_h),
                                    flags=cv2.INTER_NEAREST, borderValue=127)
            self.warped = warped.copy()

            # Place map1 on canvas
            canvas = np.full((canvas_h, canvas_w), 127, dtype=np.uint8)
            x_offset = -x_min
            y_offset = -y_min
            canvas[y_offset:y_offset + h1, x_offset:x_offset + w1] = self.map1_img

            # Merge logic
            canvas[(canvas == 255) & (warped != 0) & (canvas != 0)] = 255
            canvas[(warped == 255) & (canvas != 0)] = 255
            canvas[(canvas < 100) | (warped < 100)] = 0

            # Robot 1 position (in world coords)
            self.robot1_pos = (
                (-self.map1_info.origin.position.x) + x_offset * res,
                (-self.map1_info.origin.position.y) + y_offset * res
            )
            # Robot 2 position: transform origin of map2
            origin_px = np.array([[-self.map2_info.origin.position.x / res,
                                -self.map2_info.origin.position.y / res]], dtype=np.float32)
            transformed_px = cv2.transform(origin_px[None, :, :], M_offset)[0][0]
            self.robot2_pos = (transformed_px[0] * res, transformed_px[1] * res)

            # Rotation angle for TF (robot2)
            theta = math.atan2(M[1, 0], M[0, 0])
            self.robot2_yaw = theta

            # Inflate the walls
            #kernel = np.ones((3, 3), np.uint8)
            kernel = np.array([
                [0, 1, 0],
                [1, 1, 1],
                [0, 1, 0]
            ], dtype=np.uint8)
            canvas = cv2.erode(canvas, kernel, iterations=1)

        self.merged_map_img = canvas.copy()

        merged_msg = OccupancyGrid()
        merged_msg.header = Header()
        merged_msg.header.stamp = self.get_clock().now().to_msg()
        merged_msg.header.frame_id = 'world'
        merged_msg.info.resolution = res
        merged_msg.info.width = canvas.shape[1]
        merged_msg.info.height = canvas.shape[0]
        merged_msg.info.origin.position.x = 0.0
        merged_msg.info.origin.position.y = 0.0
        merged_msg.info.origin.position.z = 0.0
        merged_msg.info.origin.orientation.w = 1.0

        ros_data = np.full(canvas.shape, -1, dtype=np.int8)
        ros_data[canvas == 255] = 0
        ros_data[canvas == 0] = 100
        merged_msg.data = ros_data.flatten().tolist()

        self.map_publisher.publish(merged_msg)

    # Add small images to the top row of the main image
    def add_small_pictures(self, img, small_images, width=100):

        if len(img.shape) == 2:
            img = np.dstack((img, img, img))

        x_base_offset = 10
        y_base_offset = 10

        x_offset = x_base_offset
        y_offset = y_base_offset

        for small in small_images:
            if small is None:
                continue

            # Get original dimensions
            original_height, original_width = small.shape[:2]
            # Calculate the new height to maintain aspect ratio
            scale_factor = width / original_width
            new_height = int(original_height * scale_factor)
            # Resize the image
            small = cv2.resize(small, (width, new_height), interpolation=cv2.INTER_AREA)

            if len(small.shape) == 2:
                small = np.dstack((small, small, small))

            img[y_offset: y_offset + new_height, x_offset: x_offset + width] = small

            # Draw a red border around the pasted image
            top_left = (x_offset, y_offset)
            bottom_right = (x_offset + width - 1, y_offset + new_height - 1)
            border_color = (0, 0, 255)  # Red in BGR format
            border_thickness = 2  # You can adjust thickness

            cv2.rectangle(img, top_left, bottom_right, border_color, thickness=border_thickness)

            x_offset += width + x_base_offset

        return img

    def visualization_loop(self):

        #cv2.namedWindow("Map merger", cv2.WINDOW_NORMAL)

        while rclpy.ok():

            if self.merged_map_img is not None:
                # Get original dimensions
                original_height, original_width = self.merged_map_img.shape[:2]
                # Desired width
                new_width = 600
                # Calculate the new height to maintain aspect ratio
                scale_factor = new_width / original_width
                new_height = int(original_height * scale_factor)
                # Resize the image
                resized_merged_map = cv2.resize(self.merged_map_img, (new_width, new_height), interpolation=cv2.INTER_AREA)
                                
                result = self.add_small_pictures(resized_merged_map, [self.map1_img, self.proc1_img, self.map2_img, self.proc2_img, self.warped], width = 100)

                cv2.imshow("Map merger", result)
            

            if cv2.waitKey(30) & 0xFF == ord('q'):
                rclpy.shutdown()
                break
            time.sleep(1.0)


def main(args=None):
    rclpy.init(args=args)
    node = MultiRobotMapMerger()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
