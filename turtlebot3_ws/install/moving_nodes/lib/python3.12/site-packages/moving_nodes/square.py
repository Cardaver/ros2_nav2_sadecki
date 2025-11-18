#!/usr/bin/env python3
#uruchamianie jako skrypt za pomoca python3
import math #dla pi

from geometry_msgs.msg import TwistStamped #poruszanie sie
from sensor_msgs.msg import LaserScan #skanowanie
import rclpy #ros
from rclpy.node import Node

class SquareMove(Node):
    def __init__(self):
        super().__init__('square_trajectory') #konstruktor klasy Node

        self.period = 0.1 #okres
        self.buf_size = 10 #bufor wiadomosci

        self.length = 1.0 #dlugosc boku
        self.lin_vel = 0.5 #szybkosc liniowa
        self.ang_vel = 0.3 #szybkosc katowa

        self.timer = self.create_timer(self.period, self.move) #inicjalizacja timera
        
        self.dist = 0.0 #przejechany dystans
        self.angle = 0.0 #obrocony kat
        self.state = "Straight" #Straight & Rotate #rodzaj ruchu
        self.side = 0 #zrobione boki

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

        if self.state == "Straight": #jesli prosto, to jedz dopoki nie zrobisz boku
            if self.dist < self.length:
                tw.twist.linear.x = self.lin_vel
                self.dist += self.lin_vel * self.period #dodawaj przejechany dystans
            else:
                self.side += 1 #zrobiono bok
                self.dist = 0.0
                self.angle = 0.0 #stopuj
                if self.side < 4:
                    self.state = "Rotate" #jezeli nie zrobiles kwadrata to kontynuuuj
                else:
                    self.get_logger().info("Square completed.") #kwadrat zrobiony
                    self.timer.cancel()

        elif self.state == "Rotate": #obracaj sie o 90 stopni z predkoscia katowa
            if self.angle < math.pi / 2:
                tw.twist.angular.z = self.ang_vel
                self.angle += self.ang_vel * self.period #sumuj kat
            else:
                self.state = "Straight" #jesli juz sie obrociles to jedziesz dalej
                self.dist = 0.0
                self.angle = 0.0

        self.publisher.publish(tw) #publikuj ruch
        self.get_logger().info(f"Side: {self.side}, Moving: {self.state}, x = {tw.twist.linear.x:.3f},\
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
    node = SquareMove()

    rclpy.spin(node)
    node.stop_msg()
    node.destroy_node()
    rclpy.shutdown()

if __name__=="__main__":
    main()