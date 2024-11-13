#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from pysabertooth import Sabertooth
import serial

class SabertoothDriverNode(Node):
    def __init__(self):
        super().__init__('sabertooth_driver_node')
        self.subscription_cmd_vel = self.create_subscription(Twist,'cmd_vel',self.cmd_vel_callback,10)
        self.subscription_odom = self.create_subscription(Odometry,'odom',self.odom_callback,10)
        self.saber = None
        self.linear_speed = 0.0
        self.angular_speed = 0.0

        try:
            # Inicializa el controlador Sabertooth
            self.saber = Sabertooth('/dev/ttyACM0', baudrate=115200, address=128, timeout=0.1)
            self.get_logger().info('Sabertooth initialized successfully')
        except serial.SerialException as e:
            self.get_logger().error(f'Error initializing Sabertooth: {e}')
            rclpy.shutdown()

        # Parmetros del control PID para la velocidad angular
        self.Kp = 1.0  # Ganancia proporcional
        self.Ki = 0.0  # Ganancia integral
        self.Kd = 0.0  # Ganancia derivativa
        self.error_integral = 0.0
        self.last_error = 0.0

    def cmd_vel_callback(self, msg):
        self.linear_speed = msg.linear.x
        self.angular_speed = msg.angular.z

        # Convierte las velocidades a valores que acepta el Sabertooth
        left_speed = self.convert_to_sabertooth_speed(self.linear_speed - self.angular_speed)
        right_speed = self.convert_to_sabertooth_speed(self.linear_speed + self.angular_speed)

        try:
            self.saber.drive(1, left_speed)
            self.saber.drive(2, right_speed)
            self.get_logger().info(f'Driving: left_speed={left_speed}, right_speed={right_speed}')
        except serial.SerialException as e:
            self.get_logger().error(f'Error writing to Sabertooth: {e}')

    def convert_to_sabertooth_speed(self, speed):
        return max(min(int(speed * 50), 50), -50)

    def odom_callback(self, msg):
        # Asumimos que la velocidad angular estan en msg.twist.twist.angular.z
        current_angular_speed = msg.twist.twist.angular.z

        # Calcula el error de velocidad angular respecto al objetivo
        error_angular = self.angular_speed - current_angular_speed

        # Calcula la salida del PID para ajustar las velocidades de las ruedas
        pid_output = self.pid_output(error_angular)
        left_target_speed = self.linear_speed - pid_output
        right_target_speed = self.linear_speed + pid_output

        # Aplica los valores corregidos al Sabertooth
        try:
            self.saber.drive(1, self.convert_to_sabertooth_speed(left_target_speed))
            self.saber.drive(2, self.convert_to_sabertooth_speed(right_target_speed))
            self.get_logger().info(f'Driving corrected: left_speed={left_target_speed}, right_speed={right_target_speed}')
        except serial.SerialException as e:
            self.get_logger().error(f'Error writing to Sabertooth: {e}')

    def pid_output(self, error):
        # PID 
        self.error_integral += error
        derivative = error - self.last_error
        self.last_error = error
        return self.Kp * error + self.Ki * self.error_integral + self.Kd * derivative

def main():
    rclpy.init()
    sabertooth_driver_node = SabertoothDriverNode()
    rclpy.spin(sabertooth_driver_node)
    sabertooth_driver_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
