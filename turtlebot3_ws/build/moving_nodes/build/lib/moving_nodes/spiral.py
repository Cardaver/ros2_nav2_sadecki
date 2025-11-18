#!/usr/bin/env python3
#uruchamianie jako skrypt za pomoca python3
import math #dla pi

from geometry_msgs.msg import TwistStamped #poruszanie sie
from sensor_msgs.msg import LaserScan #skanowanie
import rclpy #ros
from rclpy.node import Node

class SpiralMove(Node):
    def __init__(self):
        super().__init__('circular_trajectory') #konstruktor klasy Node

        self.period = 0.1 #okres
        self.buf_size = 10 #bufor wiadomosci

        #r = a*theta + b
        self.a = 0.05 #tempo wzrostu spirali 
        self.b = 0.5 #promien okregu poczatkowy
        self.lin_vel = 0.5 #szybkosc liniowa
        self.ang_vel = self.lin_vel / self.b #szybkosc katowa

        self.timer = self.create_timer(self.period, self.move) #inicjalizacja timera
        
        self.angle = 0.0 #obrocony kat
        self.goal = 2 # liczba okrazen do zrobienia

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
        
        r = self.a * self.angle + self.b
        self.ang_vel = self.lin_vel / r

        if self.angle >= self.goal * 2 * math.pi:
            self.get_logger().info("Spiral completed.")
            self.timer.cancel()
            self.stop_msg()
            return
        
        tw.twist.linear.x = self.lin_vel
        tw.twist.angular.z = self.ang_vel

        self.publisher.publish(tw) #publikuj ruch
        self.get_logger().info(f"ang_vel = {self.ang_vel}, angle = {self.angle}")

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
    node = SpiralMove()

    rclpy.spin(node)
    node.stop_msg()
    node.destroy_node()
    rclpy.shutdown()

if __name__=="__main__":
    main()