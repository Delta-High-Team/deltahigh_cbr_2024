#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException
from pymavlink import mavutil
import time
import numpy as np
from scipy.spatial.transform import Rotation as R
#ros2 run tf2_ros tf2_echo map ldlidar_link - visualizar as transformações
#ros2 launch ldlidar_node ldlidar_slam.launch.py 

# Definir valores para a mensagem GPS_INPUT
time_usec = int(time.time() * 1e6)  # Timestamp em microssegundos
gps_id = 0 # ID do GPS
ignore_flags = 0  # Ativa todos os flags de ignorar
time_week_ms = 0  # Tempo da semana GPS em milissegundos
time_week = 0  # Número da semana GPS
fix_type = 3  # Tipo de correção (3 = RTK fix)
lat = -275937984  # Latitude em graus * 1e7
lon = -485427360  # Longitude em graus * 1e7
alt = 0 # Altitude em metros
hdop = 1.21  # Precisão horizontal diluição de posição
vdop = 2.0  # Precisão vertical diluição de posição
vn = -0.03  # Velocidade norte em m/s
ve = 0.03  # Velocidade leste em m/s
vd = 0.0  # Velocidade para baixo em m/s
speed_accuracy = 0.04  # Precisão da velocidade em m/s
horiz_accuracy = 0.3  # Precisão horizontal em metros
vert_accuracy = 0.3  # Precisão vertical em metros
yaw = 0  # Yaw em centigrados


class TF2Listener(Node):
    def __init__(self):
        super().__init__('tf2_listener_yaw')
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.timer = self.create_timer(0.1, self.on_timer)
        
        # Configuração da comunicação MAVLink
        self.mavlink_connection = mavutil.mavlink_connection('tcp:127.0.0.1:5763', source_system=1, source_component=2)
        self.mavlink_connection.wait_heartbeat()

    def on_timer(self):
        try:
            now = rclpy.time.Time()
            trans = self.tf_buffer.lookup_transform('map', 'ldlidar_link', now)
            self.get_logger().info(
                f"Translation: x = {trans.transform.translation.x}, y = {trans.transform.translation.y}, z = {trans.transform.translation.z}"
               
            )
            self.get_logger().info(
                f"Rotation: z = {trans.transform.rotation.z}, w = {trans.transform.rotation.w}"
            )
            
            
            # Enviando informações de odometria via MAVLink
            self.send_yaw(trans.transform.rotation.w,
                               trans.transform.rotation.z)
            
        except (LookupException, ConnectivityException, ExtrapolationException) as e:
            self.get_logger().warn(f"Could not transform 'map' to 'ldlidar_link': {e}")
    





    def send_yaw(self, w, qz):

        quaternion = [0.0, 0.0, qz, w]
        
        norm  = np.linalg.norm(quaternion)

        quaternion = [x/ norm for x in quaternion]
        rotation = R.from_quat(quaternion)
        euler = rotation.as_euler('xyz', degrees=True)
        yaw_value = euler[2]

        if yaw_value < 0:
            yaw_value += 360

        yaw_value = yaw_value*100
        print(yaw_value/100)
        # Criação e envio da mensagem de odometria
        self.mavlink_connection.mav.gps_input_send(
        time_usec, gps_id, ignore_flags, time_week_ms, time_week, fix_type,
        lat, lon, alt, hdop, vdop, vn, ve, vd, speed_accuracy,
        horiz_accuracy, vert_accuracy, satellites_visible=10, yaw=int(yaw_value)
    )
    

def main(args=None):
    rclpy.init(args=args)
    node = TF2Listener()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
