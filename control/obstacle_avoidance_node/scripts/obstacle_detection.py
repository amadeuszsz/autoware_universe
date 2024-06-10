#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import math
import numpy as np

from std_msgs.msg import String, Bool 
from sensor_msgs.msg import LaserScan 
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped

from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point

from scipy.ndimage import label


from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

custom_qos = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
    history=HistoryPolicy.KEEP_LAST,
    depth=5
)

class ObstacleDetection(Node):

    def __init__(self):
        super().__init__('obstacle_detection')
        self.subscription = self.create_subscription(
            LaserScan,
            '/sensing/lidar/scan',
            self.lidar_callback,
            qos_profile=custom_qos)
        self.publisher_bool = self.create_publisher(Bool, '/obstackle_detected', 10)
        self.subscription  # prevent unused variable warning
        self.subscription_map = self.create_subscription(
            OccupancyGrid,
            '/map',
            self.map_callback,
            10)
        self.subscription_pose = self.create_subscription(
            PoseStamped,
            '/localization/cartographer/pose',
            self.pose_callback,
            10)
        self.publisher_ = self.create_publisher(LaserScan, '/filtered_lidar_scan', 10)
        self.marker_pub = self.create_publisher(Marker, 'visualization_marker', 10)
        self.pub = self.create_publisher(OccupancyGrid, '/modified_map',  0)

        self.current_map = None
        self.current_pose = None


 
    def euler_from_quaternion(self, x, y, z, w):
        """
        Convert a quaternion into euler angles (roll, pitch, yaw)
        roll is rotation around x in radians (counterclockwise)
        pitch is rotation around y in radians (counterclockwise)
        yaw is rotation around z in radians (counterclockwise)
        """
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll_x = math.atan2(t0, t1)
    
        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch_y = math.asin(t2)
    
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw_z = math.atan2(t3, t4)
    
        return roll_x, pitch_y, yaw_z # in radians

    def get_yaw_from_quaternion(self, x, y, z, w):
        _, _, yaw = self.euler_from_quaternion(x, y, z, w)
        return yaw

    def map_callback(self, msg):
        self.process_occupancy_grid(msg)
        self.current_map = msg
        
        

    def pose_callback(self, msg):
        self.current_pose = msg

    def process_occupancy_grid(self, occupancy_grid):
        # Konwersja danych mapy do tablicy numpy
        width = occupancy_grid.info.width
        height = occupancy_grid.info.height
        grid = np.array(occupancy_grid.data, dtype=np.int8).reshape((height, width))

        # Label connected components using 8-connectivity
        structure = np.ones((3, 3), dtype=np.int8)  # 8-connectivity
        labeled, num_features = label(grid > 0, structure=structure)

        # Remove clusters with size between 10 and 20
        for i in range(1, num_features + 1):
            cluster_size = (labeled == i).sum()
            if 10 < cluster_size < 20:
                grid[labeled == i] = 0
        # Zamiana tablicy numpy z powrotem na listę
        occupancy_grid_new = OccupancyGrid()
        occupancy_grid_new = occupancy_grid
        occupancy_grid_new.data = grid.astype(np.int8).flatten().tolist()

        # Publikowanie zmodyfikowanej mapy
        self.pub.publish(occupancy_grid_new)
        self.current_map = occupancy_grid_new
        


    def lidar_callback(self, msg):
        if self.current_map is None or self.current_pose is None:
            return

        # Kopia danych LiDAR
        filtered_scan = LaserScan()
        filtered_scan.header = msg.header
        filtered_scan.angle_min = msg.angle_min
        filtered_scan.angle_max = msg.angle_max
        filtered_scan.angle_increment = msg.angle_increment
        filtered_scan.time_increment = msg.time_increment
        filtered_scan.scan_time = msg.scan_time
        filtered_scan.range_min = msg.range_min
        filtered_scan.range_max = msg.range_max
        filtered_scan.ranges = msg.ranges[:]
        filtered_scan.intensities = msg.intensities[:]
        
        # Przygotowanie danych do transformacji
        map_resolution = self.current_map.info.resolution
        map_origin = self.current_map.info.origin
        pose_x = self.current_pose.pose.position.x
        pose_y = self.current_pose.pose.position.y
        quat_x = self.current_pose.pose.orientation.x
        quat_y = self.current_pose.pose.orientation.y
        quat_z = self.current_pose.pose.orientation.z
        quat_w = self.current_pose.pose.orientation.w
        pose_yaw = self.get_yaw_from_quaternion(quat_x, quat_y, quat_z, quat_w)

        marker = Marker()
        marker.header.frame_id = "/map"
        marker.type = marker.POINTS
        marker.action = marker.ADD
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.04  # rozmiar punktów
        marker.scale.y = 0.04
        marker.color.a = 1.0  # Przezroczystość
        marker.color.r = 1.0  # Kolor czerwony
        marker.color.g = 1.0
        marker.color.b = 0.0

        lidar_offset_x = 0.35  # przesunięcie LiDAR od środka robota w metrach
        lidar_offset_y = 0.0  # brak przesunięcia w osi y

        # Oblicz współrzędne przesuniętego LiDAR
        lidar_x = pose_x + lidar_offset_x * np.cos(pose_yaw) 
        lidar_y = pose_y + lidar_offset_x * np.sin(pose_yaw) 



        # Filtracja skanu
        for i in range(len(filtered_scan.ranges)):
            range_val = filtered_scan.ranges[i]
            if range_val == float('Inf'):
                continue
            angle = msg.angle_min + i * msg.angle_increment
            # Przekształcenie do układu mapy
            # print(f'angle {angle} range_val {range_val} pose_yaw {pose_yaw}')
            # jak wjezdamy w sciane to wtedy jest nan
            if range_val != range_val:
                continue
            x = lidar_x + range_val * np.cos(angle + pose_yaw)
            y = lidar_y + range_val * np.sin(angle + pose_yaw)
            
            # Przekształcenie do współrzędnych mapy
            # print(f'x {x} map_origin.position.x) {map_origin.position.x}) / map_resolution/ {map_resolution}')
            map_x = int((x - map_origin.position.x) / map_resolution)
            map_y = int((y - map_origin.position.y) / map_resolution)
            # print(f'map_x {map_x} map_y {map_y}  x/ {(x - map_origin.position.x) / map_resolution}')

            p = Point()
            p.x = map_x * self.current_map.info.resolution + self.current_map.info.origin.position.x
            p.y = map_y * self.current_map.info.resolution + self.current_map.info.origin.position.y
            p.z = 0.0
            marker.points.append(p)
            
            # Sprawdzenie czy punkt znajduje się na ścianie
            # if 0 <= map_x < self.current_map.info.width and 0 <= map_y < self.current_map.info.height:
            #     index = map_y * self.current_map.info.width + map_x
            #     if self.current_map.data[index] > 10:  # założenie, że wartości >50 oznaczają zajętość
            #         filtered_scan.ranges[i] = float('Inf')  # ustawienie zasięgu na nieskończoność

            is_occupied = (
                0 <= map_x < self.current_map.info.width and 0 <= map_y < self.current_map.info.height and
                self.current_map.data[map_y * self.current_map.info.width + map_x] > 10  and 
                self.current_map.data[map_y * self.current_map.info.width + map_x] == -1# Check current cell
            )
            for neighbor_x, neighbor_y in [
                (-1, 0), (1, 0), (0, -1), (0, 1), (-1, -1), (-1, 1), (1, -1), (1, 1)
            ]:
                neighbor_map_x = map_x + neighbor_x
                neighbor_map_y = map_y + neighbor_y
                if 0 <= neighbor_map_x < self.current_map.info.width and 0 <= neighbor_map_y < self.current_map.info.height:
                    is_occupied |= self.current_map.data[neighbor_map_y * self.current_map.info.width + neighbor_map_x] > 10


            

            if is_occupied:
                filtered_scan.ranges[i] = float('Inf')  # Set range to infinity if occupied

                # to ma byc odszumianie pojedynczych skanow  

        # for i in range(len(filtered_scan.ranges)):
        #     range_val = filtered_scan.ranges[i]
        #     if range_val == float('Inf'):  # Skip already marked points
        #         continue
        #     angle = msg.angle_min + i * msg.angle_increment

        #     # Transform to map coordinates (same as before)
        #     x = pose_x + range_val * np.cos(angle + pose_yaw)
        #     y = pose_y + range_val * np.sin(angle + pose_yaw)
        #     map_x = int((x - map_origin.position.x) / map_resolution)
        #     map_y = int((y - map_origin.position.y) / map_resolution)

        #     # Check if the point has any valid neighbors
        #     has_valid_neighbors = False
        #     for neighbor_x, neighbor_y in [
        #         (-1, 0), (1, 0), (0, -1), (0, 1), (-1, -1), (-1, 1), (1, -1), (1, 1)
        #     ]:
        #         neighbor_map_x = map_x + neighbor_x
        #         neighbor_map_y = map_y + neighbor_y
        #          # Check if neighbor is within bounds before accessing
        #         if 0 <= neighbor_map_x < map_width and 0 <= neighbor_map_y < map_height:
        #             if isolated_scan.ranges[neighbor_map_y * self.current_map.info.width + neighbor_map_x] != float('Inf'):
        #                 has_valid_neighbors = True
        #                 break
        #         # if 0 <= neighbor_map_x < self.current_map.info.width and 0 <= neighbor_map_y < self.current_map.info.height:
        #         #     if filtered_scan.ranges[neighbor_map_y * self.current_map.info.width + neighbor_map_x] != float('Inf'):
        #         #         has_valid_neighbors = True
        #         #         break

        #     # Mark isolated points as occupied in the isolated scan
        #     if not has_valid_neighbors:
        #         filtered_scan.ranges[i] = float('Inf')

        
        # Publikacja przefiltrowanego skanu
        self.publisher_.publish(filtered_scan)

        self.marker_pub.publish(marker)
        obstacle_bool = False
        for i in range(len(filtered_scan.ranges)):
            distance = filtered_scan.ranges[i]
            if distance < 1.7:  
                self.get_logger().info(f'Obstacle detected at {distance} meters at angle {filtered_scan.angle_min + i * filtered_scan.angle_increment}')
                obstacle_bool = True

        # self.get_logger().info('I heard' )
        msg_bool = Bool()
        msg_bool.data = obstacle_bool
        self.publisher_bool.publish(msg_bool)
        # self.get_logger().info(f'Publishing: {msg.data}')


def main(args=None):
    rclpy.init(args=args)

    obstacle_detection = ObstacleDetection()

    rclpy.spin(obstacle_detection)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    obstacle_detection.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()