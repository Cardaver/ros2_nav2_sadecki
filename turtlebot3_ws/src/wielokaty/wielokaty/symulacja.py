import rclpy
from rclpy.node import Node
import math
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import LaserScan
from tf_transformations import quaternion_from_euler
from nav_msgs.msg import Odometry


class MoveSimulation(Node):
    def __init__(self, sides, length):
        super().__init__("simulation_publisher")

        if sides <= 2:
            self.get_logger().error("!!!BŁĄD: Liczba boków musi być większa niż 2")
            raise ValueError

        self.sides = sides
        self.length = length
        self.side_nr = 0
        self.side_ended = True
        self.out_of_center = False
        self.angle = math.pi*(sides-2)/sides
        self.radius=length/(2*math.tan(math.pi/self.sides))
        self.obstacle_distance = 0.01
        self.obstacle_detected = False
        self.end_move_position = [0, 0, 0, 0, 0, 0]

        self.publisher=self.create_publisher(PoseStamped, "/goal_pose", 10)
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.create_subscription(Odometry, "/odom", self.odometry_callback, 10)
        self.timer = self.create_timer(0.1, self.update)
        self.get_logger().info(f"Rozpoczęto proces: {sides}-kąt, bok={length}m, promień={self.radius:.2f}, kąt wewnętrzny={(self.angle*180/math.pi):.2f}°")

    def scan_callback(self, msg):
        if msg.ranges:
            min_distance = min(msg.ranges)
            if min_distance < self.obstacle_distance:
                if not self.obstacle_detected:
                    self.get_logger().warn(f"Wykryto przeszkodę w odległości mniejszej niż {self.obstacle_distance}m")
                self.obstacle_detected = True
            else:
                self.obstacle_detected = False

    def odometry_callback(self, msg):
        current_position = msg.pose.pose

        delta_x = abs(current_position.position.x-self.end_move_position[0])
        delta_y = abs(current_position.position.y-self.end_move_position[1])
        delta_position = math.sqrt(delta_x**2 + delta_y**2)

        x = current_position.orientation.x
        y = current_position.orientation.y
        z = current_position.orientation.z
        w = current_position.orientation.w
        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        current_yaw = math.atan2(siny_cosp, cosy_cosp)
        delta_orientation = abs(current_yaw-(self.side_nr*(math.pi-self.angle) - self.angle/2))

        self.get_logger().info(f"delta_x={delta_x}, delta_y={delta_y}", throttle_duration_sec=5.0)
        if delta_position < 0.05 and delta_orientation < 0.05:
            if not self.side_ended:
                self.side_ended = True
                self.get_logger().info(f"Wierzchołek {self.side_nr} został osiągnięty")

    def update(self):
        self.get_logger().info(f"Status: side_ended={self.side_ended}, out_of_center={self.out_of_center}, side_nr={self.side_nr}", throttle_duration_sec=5.0)

        if self.obstacle_detected: 
            self.get_logger().info("Obstacle detected")
            return
        
        if (self.side_ended):
            msg=PoseStamped()
            msg.header.stamp = self.get_clock().now().to_msg()
            # msg.header.frame_id = "map"
            msg.header.frame_id = "odom"
            
            if (not self.out_of_center):
                self.out_of_center = True
                self.side_ended = False

                msg.pose.position.x = -self.radius
                
                quaternion = quaternion_from_euler(0, 0, -self.angle/2)
                msg.pose.orientation.x = quaternion[0]
                msg.pose.orientation.y = quaternion[1]
                msg.pose.orientation.z = quaternion[2]
                msg.pose.orientation.w = quaternion[3]

                self.end_move_position = [-self.radius, 0, quaternion[0], quaternion[1], quaternion[2], quaternion[3]]
                
                self.publisher.publish(msg)
                self.get_logger().info("Rozpoczęto jazdę do figury")
            else:
                if (self.side_nr < self.sides*2):
                    self.side_nr+=1
                    self.side_ended = False

                    msg.pose.position.x = math.cos(self.angle)*self.end_move_position[0] - math.sin(self.angle)*self.end_move_position[1]
                    msg.pose.position.y = math.sin(self.angle)*self.end_move_position[0] + math.cos(self.angle)*self.end_move_position[1]
                    
                    new_angle = self.side_nr*(math.pi-self.angle) - self.angle/2 
                    quaternion = quaternion_from_euler(0, 0, new_angle)
                    msg.pose.orientation.x = quaternion[0]
                    msg.pose.orientation.y = quaternion[1]
                    msg.pose.orientation.z = quaternion[2]
                    msg.pose.orientation.w = quaternion[3]

                    self.end_move_position = [msg.pose.position.x, msg.pose.position.y, quaternion[0], quaternion[1], quaternion[2], quaternion[3]]
                    
                    self.publisher.publish(msg)
                    self.get_logger().info(f"Rozpoczęto {self.side_nr} bok wielokąta foremnego")
                else:
                    self.get_logger().info(f"Figura foremna {self.sides}-kątna została ukończona")
                    self.timer.cancel()


def main(args=None):
    rclpy.init(args=args)
    try:
        node = MoveSimulation(4,1)
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception:
        pass
    finally:
        stop_msg = PoseStamped()
        stop_msg.header.stamp = node.get_clock().now().to_msg()
        stop_msg.header.frame_id = "map"
        #stop_msg.header.frame_id = "odom"
        node.publisher.publish(stop_msg)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()