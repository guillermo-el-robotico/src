#!/usr/bin/env python3
import serial
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
from tf_transformations import quaternion_from_euler
from sensor_msgs.msg import JointState, BatteryState

class OdometryPublisher(Node):

    def __init__(self):
        super().__init__('odometry_publisher')

        # Configuracion del puerto serialx
        self.ser = serial.Serial('/dev/ttyACM1', 9600)  # Cambia tu puerto
        self.ser.flush()

        # Variables para almacenar x, y, theta, linear, angular, corriente y voltaje
        self.pos_x = 0
        self.pos_y = 0
        self.theta = 0
        self.linear = 0
        self.angular = 0
        self.corriente = 0
        self.voltaje = 0
        self.R = 0.07
        self.d = 0.38
        self.wR = 0
        self.wL = 0

        # Publicador de Odometry
        self.odom_pub_ = self.create_publisher(Odometry, "odom", 10)

        # Publicador de JointState
        self.joint_state_pub_ = self.create_publisher(JointState, "joint_states", 10)

        # Publicador de Corriente
        self.corriente_pub_ = self.create_publisher(BatteryState, "corriente", 10)

        # Publicador de Voltaje
        self.voltaje_pub_ = self.create_publisher(BatteryState, "voltaje", 10)

        # TransformBroadcaster
        self.br_ = TransformBroadcaster(self)

        while True:
            try:
                # Leer datos de la conexion serial
                data = self.ser.readline().decode('utf-8').strip()
               
                # Dividir los datos en una lista de valores
                values = data.split(',')

                # Verificar si se han recibido todos los valores esperados
                if len(values) != 7:
                    self.get_logger().error(f"Datos incompletos recibidos: {data}")
                    continue

                # Convertir los valores a numeros flotantes
                x = float(values[0])
                y = float(values[1])
                theta = float(values[2])
                linear = float(values[3])
                angular = float(values[4])
                corriente = float(values[5])
                voltaje = float(values[6])

                self.get_logger().info(
                    f"Posición X: {x:.2f} m, "
                    f"Posición Y: {y:.2f} m, "
                    f"Orientación Theta: {theta:.3f} rad, "
                    f"Velocidad lineal: {linear:.1f} m/s, "
                    f"Velocidad angular: {angular:.1f} rad/s, "
                    f"Corriente: {corriente:.2f} A, "
                    f"Voltaje: {voltaje:.2f} V"
                )

                self.pos_x = x
                self.pos_y = y
                self.theta = theta
                self.linear = linear
                self.angular = angular
                self.corriente = corriente
                self.voltaje = voltaje

                self.odom_msg = Odometry()
                self.odom_msg.header.frame_id = "odom"
                self.odom_msg.child_frame_id = "base_footprint"
                self.odom_msg.header.stamp = self.get_clock().now().to_msg()

                # Llenar el mensaje de odometria
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

                # Publicar el mensaje de odometria
                self.odom_pub_.publish(self.odom_msg)

                self.transform_stamped_ = TransformStamped()
                self.transform_stamped_.header.frame_id = "odom"
                self.transform_stamped_.child_frame_id = "base_footprint"
                self.transform_stamped_.transform.translation.x = self.pos_x
                self.transform_stamped_.transform.translation.y = self.pos_y
                self.transform_stamped_.transform.translation.z = 0.0
                self.transform_stamped_.header.stamp = self.get_clock().now().to_msg()

                # Añadir la orientacion al transformado
                self.transform_stamped_.transform.rotation.x = q[0]
                self.transform_stamped_.transform.rotation.y = q[1]
                self.transform_stamped_.transform.rotation.z = q[2]
                self.transform_stamped_.transform.rotation.w = q[3]

                # Publicar el TF
                self.br_.sendTransform(self.transform_stamped_)

                self.wR = ((self.linear * 2) / self.R) - self.wL
                self.wL = ((self.angular * self.d) / self.R) + self.wR
                joint_state_msg = JointState()
                joint_state_msg.header.stamp = self.get_clock().now().to_msg()
                joint_state_msg.name = ["left_wheel_joint", "right_wheel_joint"]
                joint_state_msg.position = [self.pos_x, self.pos_y]  # Suponiendo que los valores del encoder se usan para posiciones
                joint_state_msg.velocity = [self.wL, self.wR]  # Suponiendo que los valores del encoder se usan para velocidades

                # Publicar el mensaje de JointState
                self.joint_state_pub_.publish(joint_state_msg)

                # Publicar el valor de corriente
                corriente_msg = BatteryState()
                corriente_msg.header.stamp = self.get_clock().now().to_msg()
                corriente_msg.current = self.corriente

                self.corriente_pub_.publish(corriente_msg)

                # Publicar el valor de voltaje
                voltaje_msg = BatteryState()
                voltaje_msg.header.stamp = self.get_clock().now().to_msg()
                voltaje_msg.voltage = self.voltaje

                self.voltaje_pub_.publish(voltaje_msg)

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
    main()  #  xxxxxxxxxxxxxxxxxxxx
