#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import serial
import time

class CmdVelToSerial(Node):
    def __init__(self):
        super().__init__('cmd_vel_to_serial')

        # Suscribirse al topico /cmd_vel
        self.subscription = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10)
        
        # Configuracion de la comunicacion serial
        self.serial_port = '/dev/ttyACM0'  # Cambia esto segun tu puerto serial
        self.baud_rate = 9600              # Asegurate de que coincida con el Arduino
        self.serial_conn = None
        
        try:
            self.serial_conn = serial.Serial(self.serial_port, self.baud_rate, timeout=1)
            self.get_logger().info(f'Conectado al puerto serial: {self.serial_port}')
        except serial.SerialException as e:
            self.get_logger().error(f'No se pudo conectar al puerto serial: {e}')
        
        # Variables para almacenar los valores recibidos
        self.linear_x = 0.0
        self.angular_z = 0.0

    def cmd_vel_callback(self, msg):
        # Extraer valores de /cmd_vel
        self.linear_x = msg.linear.x
        self.angular_z = msg.angular.z
        self.get_logger().info(f'Recibido - Linear.x: {self.linear_x}, Angular.z: {self.angular_z}')
        
        # Enviar los datos por serial
        self.send_to_serial(self.linear_x, self.angular_z)

    def send_to_serial(self, linear_x, angular_z):
        if self.serial_conn and self.serial_conn.is_open:
            try:
                # Crear el mensaje en formato: "LX:<valor>,AZ:<valor>"
                serial_msg = f"LX:{linear_x:.2f},AZ:{angular_z:.2f}\n"
                self.serial_conn.write(serial_msg.encode('utf-8'))
                self.get_logger().info(f'Enviado por serial: {serial_msg}')
            except serial.SerialException as e:
                self.get_logger().error(f'Error enviando por serial: {e}')
        else:
            self.get_logger().warning('Conexion serial no disponible')

    def destroy_node(self):
        # Cerrar la conexion serial al finalizar el nodo
        if self.serial_conn and self.serial_conn.is_open:
            self.serial_conn.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = CmdVelToSerial()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
