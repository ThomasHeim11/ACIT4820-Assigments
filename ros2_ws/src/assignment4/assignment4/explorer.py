import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped, Point, Quaternion
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from action_msgs.msg import GoalStatus
import numpy as np
import tf2_ros
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException
import math
from collections import deque
from visualization_msgs.msg import Marker
from rclpy.duration import Duration


class ExplorerNode(Node):
    def __init__(self):
        super().__init__('explorer')
        self.get_logger().info("Dynamic Explorer Node Started")
        self.min_cluster = 5
        self.state = 'IDLE'
        self.waiting_log_printed = False
        # Radius around a visited goal to consider "explored"
        self.visited_radius = 0.4
        # Weight for penalizing non-forward frontiers.
        # Higher value = stronger preference for forward direction.
        self.angular_distance_weight = 0.5

        # Subscriptions, clients, and publishers
        self.map_sub = self.create_subscription(
            OccupancyGrid, '/map', self.map_callback, 10)
        self.nav_to_pose_client = ActionClient(self, NavigateToPose,
                                               'navigate_to_pose')
        self.frontier_marker_pub = self.create_publisher(Marker,
                                                         '/frontier_markers', 10)

        # Action goal and state variables
        self.current_goal_handle = None
        self.active_goal_msg = None

        # Data storage
        self.visited_frontiers = set()
        self.unreachable_frontiers = set()
        self.map_data = None
        self.robot_pose = None

        # New variables to store the start pose and track completion
        self.start_pose = None
        self.exploration_finished = False

        # TF listener for robot pose
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Main exploration loop timer
        self.timer = self.create_timer(1.0, self.explore)

    def get_robot_pose(self):
        try:
            # Get the transform from the 'map' frame to the 'base_link' frame
            transform = self.tf_buffer.lookup_transform('map', 'base_link',
                                                        rclpy.time.Time())
            self.robot_pose = transform.transform
            return True
        except (LookupException, ConnectivityException, ExtrapolationException) as e:
            self.get_logger().warn(f"Could not get robot pose: {e}")
            return False

    def map_callback(self, msg):
        self.map_data = msg

    def navigate_to(self, x, y, orientation=None):
        self.state = 'NAVIGATING'
        self.waiting_log_printed = False

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y

        # Use provided orientation or default to facing forward
        if orientation:
            goal_msg.pose.pose.orientation = orientation
        else:
            goal_msg.pose.pose.orientation.w = 1.0

        self.active_goal_msg = goal_msg

        self.get_logger().info(f"Navigating to goal: x={x:.2f}, y={y:.2f}")
        self.nav_to_pose_client.wait_for_server()
        send_goal_future = self.nav_to_pose_client.send_goal_async(goal_msg)
        send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warning("Goal rejected!")
            self.state = 'IDLE'
            self.active_goal_msg = None
            return

        self.current_goal_handle = goal_handle
        self.get_logger().info("Goal accepted.")
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.navigation_complete_callback)

    def navigation_complete_callback(self, future):
        result = future.result()
        status = result.status
        goal_pose = self.active_goal_msg.pose.pose.position
        goal_tuple = (goal_pose.x, goal_pose.y)

        # Reset state
        self.current_goal_handle = None
        self.active_goal_msg = None
        self.state = 'IDLE'

        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info("Navigation succeeded!")
            # Don't add the start point to the list of visited frontiers
            if not self.exploration_finished:
                self.visited_frontiers.add(goal_tuple)
        elif status == GoalStatus.STATUS_CANCELED:
            self.get_logger().info("Navigation was canceled.")
        else:
            self.get_logger().warning(f"Navigation failed with status code: {status}")
            if not self.exploration_finished:
                self.unreachable_frontiers.add(goal_tuple)

    def find_frontiers(self, map_array):
        frontiers = []
        rows, cols = map_array.shape
        # Iterate through the map, avoiding the edges
        for r in range(1, rows - 1):
            for c in range(1, cols - 1):
                # A frontier is a free cell (0) next to an unknown cell (-1)
                if map_array[r, c] == 0:
                    # Check 8-connected neighbors
                    if -1 in map_array[r-1:r+2, c-1:c+2].flatten():
                        frontiers.append((r, c))
        return frontiers

    def cluster_frontiers(self, frontier_points):
        clusters = []
        visited = set()
        # Convert list to set for faster lookups
        frontier_set = set(frontier_points)

        for point in frontier_points:
            if point not in visited:
                new_cluster = []
                queue = deque([point])
                visited.add(point)

                # BFS to find all connected frontier points
                while queue:
                    current_r, current_c = queue.popleft()
                    new_cluster.append((current_r, current_c))
                    for dr in [-1, 0, 1]:
                        for dc in [-1, 0, 1]:
                            if dr == 0 and dc == 0: continue
                            neighbor = (current_r + dr, current_c + dc)
                            if neighbor in frontier_set and neighbor not in visited:
                                visited.add(neighbor)
                                queue.append(neighbor)

                # Ignore small, insignificant clusters
                if len(new_cluster) > self.min_cluster:
                    clusters.append(new_cluster)
        return clusters

    def choose_best_cluster(self, clusters):
        if not clusters or self.robot_pose is None: return None

        # Get the robot's current yaw (heading) from the pose quaternion
        q = self.robot_pose.rotation
        robot_yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))

        min_dist_to_frontier = 0.5
        good_candidates, close_candidates = [], []

        for cluster in clusters:
            # Calculate cluster centroid in map coordinates
            centroid_r = sum(p[0] for p in cluster) / len(cluster)
            centroid_c = sum(p[1] for p in cluster) / len(cluster)

            # Convert centroid to world coordinates (meters)
            centroid_x = centroid_c * self.map_data.info.resolution + self.map_data.info.origin.position.x
            centroid_y = centroid_r * self.map_data.info.resolution + self.map_data.info.origin.position.y
            centroid_tuple = (centroid_x, centroid_y)

            # Filter out frontiers near previously visited or unreachable goals
            is_too_close_to_visited = False
            for visited_goal in self.visited_frontiers:
                dist_to_visited = math.sqrt((centroid_x - visited_goal[0])**2 + (centroid_y - visited_goal[1])**2)
                if dist_to_visited < self.visited_radius:
                    is_too_close_to_visited = True; break

            if is_too_close_to_visited or centroid_tuple in self.unreachable_frontiers:
                continue

            robot_x = self.robot_pose.translation.x
            robot_y = self.robot_pose.translation.y
            distance_meters = math.sqrt((robot_x - centroid_x)**2 + (robot_y - centroid_y)**2)

            # Calculate a score based on distance and angle to prioritize forward frontiers
            angle_to_frontier = math.atan2(centroid_y - robot_y, centroid_x - robot_x)
            angle_diff = abs(robot_yaw - angle_to_frontier)
            # Handle angle wrapping (e.g., difference between +pi and -pi is small)
            if angle_diff > math.pi:
                angle_diff = 2 * math.pi - angle_diff

            # The score combines distance and the weighted angular difference. A lower score is better.
            score = distance_meters + (angle_diff * self.angular_distance_weight)

            # Store the score with the centroid coordinates
            candidate = (score, (centroid_r, centroid_c))
            if distance_meters > min_dist_to_frontier:
                good_candidates.append(candidate)
            else:
                close_candidates.append(candidate)

        # Prefer 'good' candidates (not too close) first, sorted by the best score
        if good_candidates:
            good_candidates.sort(key=lambda x: x[0]); return good_candidates[0][1]
        # If no good candidates, consider the close ones, also sorted by score
        if close_candidates:
            close_candidates.sort(key=lambda x: x[0]); return close_candidates[0][1]

        return None

    def publish_frontier_markers(self, clusters):
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "frontiers"
        marker.id = 0
        marker.type = Marker.POINTS
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        # Set marker scale and color
        marker.scale.x = self.map_data.info.resolution * 2
        marker.scale.y = self.map_data.info.resolution * 2
        marker.color.g = 1.0  # Green
        marker.color.a = 1.0  # Fully opaque
        marker.points = []
        for cluster in clusters:
            for point_r, point_c in cluster:
                p = Point()
                p.x = point_c * self.map_data.info.resolution + self.map_data.info.origin.position.x
                p.y = point_r * self.map_data.info.resolution + self.map_data.info.origin.position.y
                p.z = 0.0
                marker.points.append(p)
        self.frontier_marker_pub.publish(marker)

    def explore(self):
        if self.map_data is None or not self.get_robot_pose():
            return

        # Save the robot's starting pose the first time it's available
        if self.start_pose is None and self.robot_pose is not None:
            self.start_pose = self.robot_pose
            self.get_logger().info(f"Start pose saved: x={self.start_pose.translation.x:.2f}, y={self.start_pose.translation.y:.2f}")

        # Main logic: find, cluster, and choose frontiers
        map_array = np.array(self.map_data.data).reshape(
            (self.map_data.info.height, self.map_data.info.width))
        frontiers = self.find_frontiers(map_array)
        clusters = self.cluster_frontiers(frontiers)

        if clusters:
            self.publish_frontier_markers(clusters)

        # Only look for new frontiers if exploration is not finished
        chosen_centroid = None
        if not self.exploration_finished:
            chosen_centroid = self.choose_best_cluster(clusters)

        if self.state == 'IDLE':
            if not chosen_centroid:
                # This block is entered if no valid frontiers are found
                # OR if exploration has already been marked as finished.

                # If exploration is not yet marked as finished, and we have a start pose,
                # it means we just ran out of frontiers. Time to go home.
                if not self.exploration_finished and self.start_pose is not None:
                    self.exploration_finished = True
                    self.get_logger().info("Exploration complete! Returning to the starting point.")
                    start_x = self.start_pose.translation.x
                    start_y = self.start_pose.translation.y
                    start_orientation = self.start_pose.rotation
                    self.navigate_to(start_x, start_y, start_orientation)

                # If we are already returning, or can't return, just wait.
                elif not self.waiting_log_printed:
                    self.get_logger().info("No valid frontiers found. Waiting...")
                    self.waiting_log_printed = True
                    # Clear markers if no frontiers are found
                    self.publish_frontier_markers([])
                return

            # If we are here, a valid frontier was chosen.
            self.waiting_log_printed = False  # Reset the waiting log
            goal_r, goal_c = chosen_centroid
            goal_x = goal_c * self.map_data.info.resolution + self.map_data.info.origin.position.x
            goal_y = goal_r * self.map_data.info.resolution + self.map_data.info.origin.position.y
            self.navigate_to(goal_x, goal_y)

        elif self.state == 'NAVIGATING' and not self.exploration_finished:
            # --- GOAL VALIDATION AND PREEMPTION LOGIC ---
            current_goal_x = self.active_goal_msg.pose.pose.position.x
            current_goal_y = self.active_goal_msg.pose.pose.position.y

            # 1. Check if the current goal's frontier has disappeared
            is_goal_still_valid = False
            for cluster in clusters:
                cluster_cr = sum(p[0] for p in cluster) / len(cluster)
                cluster_cc = sum(p[1] for p in cluster) / len(cluster)
                cluster_cx = cluster_cc * self.map_data.info.resolution + self.map_data.info.origin.position.x
                cluster_cy = cluster_cr * self.map_data.info.resolution + self.map_data.info.origin.position.y
                dist = math.sqrt((current_goal_x - cluster_cx)**2 + (current_goal_y - cluster_cy)**2)
                if dist < self.visited_radius:
                    is_goal_still_valid = True
                    break

            if not is_goal_still_valid:
                self.get_logger().info("Target frontier has disappeared. Canceling current goal.")
                self.current_goal_handle.cancel_goal_async()
                return

            # 2. If the goal is still valid, check if there's a *better* one
            if chosen_centroid:
                goal_r, goal_c = chosen_centroid
                goal_x = goal_c * self.map_data.info.resolution + self.map_data.info.origin.position.x
                goal_y = goal_r * self.map_data.info.resolution + self.map_data.info.origin.position.y

                dist_to_current_goal = math.sqrt((self.robot_pose.translation.x - current_goal_x)**2 + (self.robot_pose.translation.y - current_goal_y)**2)
                dist_to_new_goal = math.sqrt((self.robot_pose.translation.x - goal_x)**2 + (self.robot_pose.translation.y - goal_y)**2)

                # Preempt if a new goal is significantly closer (e.g., 20% closer)
                if dist_to_new_goal < dist_to_current_goal * 0.8:
                    self.get_logger().info(f"Found a better frontier ({dist_to_new_goal:.2f}m vs {dist_to_current_goal:.2f}m). Preempting.")
                    self.current_goal_handle.cancel_goal_async()

    def shutdown_node(self):
        # Graceful shutdown procedure
        if self.state == 'SHUTTING_DOWN': return
        self.state = 'SHUTTING_DOWN'
        self.get_logger().info("Shutting down explorer node.")
        self.timer.cancel()
        if self.current_goal_handle:
            self.get_logger().info("Cancelling active navigation goal.")
            self.current_goal_handle.cancel_goal_async()


def main(args=None):
    rclpy.init(args=args)
    explorer_node = ExplorerNode()
    try:
        rclpy.spin(explorer_node)
    except KeyboardInterrupt:
        explorer_node.get_logger().info("Exploration stopped by user.")
    finally:
        explorer_node.shutdown_node()
        explorer_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
