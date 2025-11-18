#!/usr/bin/env python3
#uruchamianie jako skrypt za pomoca python3
import math #dla pi

from geometry_msgs.msg import TwistStamped #poruszanie sie
from sensor_msgs.msg import LaserScan #skanowanie
import rclpy #ros
from rclpy.node import Node

class CircularMove(Node):
    def __init__(self):
        super().__init__('circular_trajectory') #konstruktor klasy Node

        self.period = 0.1 #okres
        self.buf_size = 10 #bufor wiadomosci

        self.radius = 1.0 #promien okregu
        self.lin_vel = 0.5 #szybkosc liniowa
        self.ang_vel = self.lin_vel / self.radius #szybkosc katowa

        self.timer = self.create_timer(self.period, self.move) #inicjalizacja timera
        
        self.dist = 0.0 #przejechany dystans
        self.angle = 0.0 #obrocony kat
        self.state = "Straight" #Straight & Rotate #rodzaj ruchu
        self.goal = 2 # liczba okrazen do zrobienia
        self.encirclement = 0 #zrobione okrazenia

        self.detected = False #wykryta przeszkoda
        self.obs_dist = 0.5 #dystans od przeszkody

        self.publisher = self.create_publisher(TwistStamped, '/cmd_vel', self.buf_size)  #publikujacy
        self.scan_subscriber = self.create_subscription(LaserScan, '/scan', self.scan_callback, self.buf_size) #subskrybujacy

    def move(self):
        tw = TwistStamped() #obiekt wprowadzajacy poruszanie sie

        if self.detected: #wykryto przeszkode - stop
            self.get_logger().info("Obstacle detected")
            self.stop_msg()
            return

        if self.encirclement < self.goal:
            if self.dist < 2 * math.pi * self.radius:
                tw.twist.linear.x = self.lin_vel
                tw.twist.angular.z = self.ang_vel
                self.dist += self.lin_vel * self.period
            else:
                self.encirclement += 1
                self.dist = 0.0
                self.get_logger().info(f"Circle {self.encirclement} completed.")

        if self.encirclement >= self.goal:
            self.get_logger().info("All circles completed.")
            self.timer.cancel()
            self.stop_msg()
            return

        self.publisher.publish(tw) #publikuj ruch
        self.get_logger().info(f"Encirclement: {self.encirclement}, Dist = {self.dist}, x = {tw.twist.linear.x:.3f},\
                               ang = {tw.twist.angular.z:.3f}")

    def scan_callback(self, msg):
        
        if msg.ranges:
            min_dist = min(msg.ranges)
            if min_dist < self.obs_dist:
                self.detected = True
            else:
                self.detected = False

    def stop_msg(self):
        st = TwistStamped()
        st.twist.linear.x = 0.0
        st.twist.angular.z = 0.0
        self.publisher.publish(st)

def main(args = None):
    rclpy.init(args = args)
    node = CircularMove()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("^C interrupt.")
        node.timer.cancel()
        node.stop_msg()
    finally:
        node.stop_msg()
        node.timer.cancel()
        node.destroy_node()
        rclpy.shutdown()

if __name__=="__main__":
    main()