#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException
from pymavlink import mavutil
import time
import numpy as np
from scipy.spatial.transform import Rotation as R
from math import pi
import math

#ros2 run tf2_ros tf2_echo map ldlidar_link - visualizar as transformações
#ros2 launch ldlidar_node ldlidar_slam.launch.py 
#export QT_QPA_PLATFORM=offscreen - Executar rviz2 sem parte gráfica

latitude=-27.593479
longitude=-48.543068
altitude=15.4
timestamp = 1717354508
Initial = True

class TF2Listener(Node):
   
    def __init__(self):
        super().__init__('tf2_pub_visual_position')
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.timer = self.create_timer(0.05, self.on_timer)
        
        # Configuração da comunicação MAVLink
        self.mavlink_connection = mavutil.mavlink_connection('127.0.0.1:14551',source_system=1, source_component=2)
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
           
            self.set_global_position()
               

            # Enviando informações de translação via MAVLink
            self.send_gps_position(trans.transform.translation.x,
                                   trans.transform.translation.y,
                                   trans.transform.translation.z,
                                   trans.transform.rotation.x,
                                   trans.transform.rotation.y,
                                   trans.transform.rotation.w,
                                   trans.transform.rotation.z)
            
            
        except (LookupException, ConnectivityException, ExtrapolationException) as e:
            self.get_logger().warn(f"Could not transform 'map' to 'ldlidar_link': {e}")
    


    def set_global_position(self):
        global Initial
        print("Heartbeat from system (system %u component %u)" % (self.mavlink_connection.target_system, 
                                                                self.mavlink_connection.target_component))
        if Initial:
            self.mavlink_connection.mav.set_gps_global_origin_send(
                        self.mavlink_connection.target_system,  
                        int(latitude*1e7),                  # Latitude in degrees
                        int(longitude*1e7),                  # Longitude in degrees
                        int(altitude),       
                        int(timestamp)
                    )
            Initial = False


    def send_gps_position(self, x, y, z, qx, qy, w, qz):

        quaternion = [qx, qy, qz, w]
        
        norm  = np.linalg.norm(quaternion)

        quaternion = [x/ norm for x in quaternion]
        rotation = R.from_quat(quaternion)
        euler = rotation.as_euler('xyz', degrees=True)
        yaw = euler[2]

        if yaw < 0:
            yaw += 360

        print(f'Yaw: {yaw}')

       
        # Constantes
        meters_per_deg_lat = 111320.0  # metros por grau de latitude (aproximado)

        # Calcular delta latitude
        delta_lat = x / meters_per_deg_lat

        # Calcular delta longitude (ajustado para a latitude inicial)
        meters_per_deg_lon = meters_per_deg_lat * math.cos(math.radians(latitude))
        delta_lon = -y / meters_per_deg_lon

        # Nova latitude e longitude
        new_latitude = latitude + delta_lat
        new_longitude = longitude + delta_lon


        # Criação e envio da mensagem local_ned com covariância
        self.mavlink_connection.mav.gps_input_send(
            int(timestamp),  # Timestamp (microseconds since UNIX epoch or system boot)
            1,     # gps_id
            255,     # ignore_flags
            0,     # time_week_ms
            0,     # time_week
            6,     # fix_type
            int(new_latitude*1e7),                 
            int(new_longitude*1e7),
            0,   # alt
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            50,
            int(yaw*100)
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
