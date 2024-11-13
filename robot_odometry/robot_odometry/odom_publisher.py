#!/usr/bin/env python3
import serial
import rclpy
from rclpy.time import Time
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TwistStamped
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
from tf_transformations import quaternion_from_euler

class OdometryPublisher(Node):

    def __init__(self):
        super().__init__('odometry_publisher')

        # Configuración del puerto serial
        self.ser = serial.Serial('/dev/ttyACM1', 9600)  # Cambia tu puerto
        self.ser.flush()

        # Variables para almacenar x, y, theta, linear y angular
        self.pos_x = 0
        self.pos_y = 0
        self.theta = 0
        self.linear = 0
        self.angular = 0

        # Publicador de Odometry
        self.odom_pub_ = self.create_publisher(Odometry, "odom", 10)

        # TransformBroadcaster
        self.br_ = TransformBroadcaster(self)

        while True:
            try:
                # Leer datos de la conexión serial
                data = self.ser.readline().decode('utf-8').strip()
               
                # Dividir los datos en una lista de valores
                values = data.split(',')

                # Verificar si se han recibido todos los valores esperados
                if len(values) != 5:
                    self.get_logger().error(f"Datos incompletos recibidos: {data}")
                    continue

                # Convertir los valores a números flotantes
                x = float(values[0])
                y = float(values[1])
                theta = float(values[2])
                linear = float(values[3])
                angular = float(values[4])

                self.get_logger().info(
                    f"Posición X: {x:.2f} m, "
                    f"Posición Y: {y:.2f} m, "
                    f"Orientación Theta: {theta:.3f} rad, "
                    f"Velocidad lineal: {linear:.1f} m/s, "
                    f"Velocidad angular: {angular:.1f} rad/s"
                )

                self.pos_x = x
                self.pos_y = y
                self.theta = theta
                self.linear = linear
                self.angular = angular

                self.odom_msg = Odometry()
                self.odom_msg.header.frame_id = "odom"
                self.odom_msg.child_frame_id = "base_footprint"
                self.odom_msg.header.stamp = self.get_clock().now().to_msg()

                # Llenar el mensaje de odometría
                q = quaternion_from_euler(0, 0, self.theta)
                self.odom_msg.pose.pose.position.x = self.pos_x
                self.odom_msg.pose.pose.position.y = self.pos_y
                self.odom_msg.pose.pose.position.z = 0.0
                self.odom_msg.pose.pose.orientation.x = q[0]
                self.odom_msg.pose.pose.orientation.y = q[1]
                self.odom_msg.pose.pose.orientation.z = q[2]
                self.odom_msg.pose.pose.orientation.w = q[3]
                self.odom_msg.twist.twist.linear.x = self.linear
                self.odom_msg.twist.twist.angular.z = self.angular

                # Publicar el mensaje de odometría
                self.odom_pub_.publish(self.odom_msg)

                self.transform_stamped_ = TransformStamped()
                self.transform_stamped_.header.frame_id = "odom"
                self.transform_stamped_.child_frame_id = "base_footprint"
                self.transform_stamped_.transform.translation.x = self.pos_x
                self.transform_stamped_.transform.translation.y = self.pos_y
                self.transform_stamped_.transform.translation.z = 0.0
                self.transform_stamped_.header.stamp = self.get_clock().now().to_msg()

                # Añadir la orientación al transformado
                self.transform_stamped_.transform.rotation.x = q[0]
                self.transform_stamped_.transform.rotation.y = q[1]
                self.transform_stamped_.transform.rotation.z = q[2]
                self.transform_stamped_.transform.rotation.w = q[3]

                # Publicar el TF
                self.br_.sendTransform(self.transform_stamped_)

            except ValueError as e:
                self.get_logger().error(f"Error al convertir a float: {e}")
            except Exception as e:
                self.get_logger().error(f"Error inesperado: {e}")

def main():
    rclpy.init()
    odometry_publisher = OdometryPublisher()
    rclpy.spin(odometry_publisher)
    odometry_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
