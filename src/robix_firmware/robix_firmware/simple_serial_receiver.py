#!/usr/bin/env python3
import serial
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Quaternion, Vector3
import math


class SimpleSerialReceiver(Node):
    def __init__(self):
        super().__init__("simple_serial_receiver")

        self.declare_parameter("wheel_radius", 0.033)
        self.declare_parameter("wheel_separation", 0.34)

        self.wheel_radius_ = self.get_parameter("wheel_radius").get_parameter_value().double_value
        self.wheel_separation_ = self.get_parameter("wheel_separation").get_parameter_value().double_value

        self.get_logger().info("Using wheel radius %d" % self.wheel_radius_)
        self.get_logger().info("Using wheel separation %d" % self.wheel_separation_)

        self.declare_parameter("port", "/dev/ttyUSB0")
        self.declare_parameter("baudrate", 115200)

        self.port_ = self.get_parameter("port").value
        self.baudrate_ = self.get_parameter("baudrate").value

        self.pub_odom_ = self.create_publisher(Odometry, "odom", 10)
        self.arduino_ = serial.Serial(port=self.port_, baudrate=self.baudrate_, timeout=0.1)

        self.frequency_ = 0.01
        self.timer_ = self.create_timer(self.frequency_, self.timerCallback)

        self.right_wheel_est_vel = 0
        self.left_wheel_est_vel = 0

        # Publish odometry message
        self.odom_msg = Odometry()
        self.odom_msg.header.frame_id = "odom"
        self.odom_msg.child_frame_id = "base_link"

    def timerCallback(self):
        if rclpy.ok() and self.arduino_.is_open:
            data = self.arduino_.readline().decode().strip()

            try:
                # Parse the received data
                right_wheel_meas_vel, left_wheel_meas_vel = map(int, data.split(","))
            except ValueError:
                # Invalid data received
                return

            # Calculate estimated velocities
            self.right_wheel_est_vel = right_wheel_meas_vel 
            self.left_wheel_est_vel =  right_wheel_meas_vel #left_wheel_meas_vel


            # Calculate linear and angular velocities
            linear = (self.right_wheel_est_vel + self.left_wheel_est_vel) / 2
            angular = (self.right_wheel_est_vel - self.left_wheel_est_vel) / self.wheel_separation_


            #self.get_logger().info("The linear is: {:.2f} and angular is: {:.2f}".format(linear,angular))
            self.odom_msg.header.stamp = self.get_clock().now().to_msg()


            # Set the linear velocity
            self.odom_msg.twist.twist.linear.x = linear

            # Set the angular velocity
            self.odom_msg.twist.twist.angular.z = angular

            self.pub_odom_.publish(odom_msg)


def main():
    rclpy.init()

    simple_serial_receiver = SimpleSerialReceiver()
    rclpy.spin(simple_serial_receiver)
    
    simple_serial_receiver.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
