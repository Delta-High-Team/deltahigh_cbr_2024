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
#ros2 run tf2_ros tf2_echo map ldlidar_link - visualizar as transformações
#ros2 launch ldlidar_node ldlidar_slam.launch.py 
#export QT_QPA_PLATFORM=offscreen - Executar rviz2 sem parte gráfica



yaw = 0  # Yaw em rad
x = 0
y=0
z=0

class TF2Listener(Node):
    def __init__(self):
        super().__init__('tf2_pub_visual_position')
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.timer = self.create_timer(0.1, self.on_timer)
        
        # Configuração da comunicação MAVLink
        self.mavlink_connection = mavutil.mavlink_connection('tcp:127.0.0.1:5762', source_system=1, source_component=2)
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
            
            
            # Enviando informações de translação via MAVLink
            self.send_vision_position_estimate(trans.transform.translation.x,
                                               trans.transform.translation.y,
                                               trans.transform.translation.z,
                                               trans.transform.rotation.w,
                                               trans.transform.rotation.z)
            
            
        except (LookupException, ConnectivityException, ExtrapolationException) as e:
            self.get_logger().warn(f"Could not transform 'map' to 'ldlidar_link': {e}")
    

    def send_vision_position_estimate(self, x, y, z, w, qz):

        quaternion = [0.0, 0.0, qz, w]
        
        norm  = np.linalg.norm(quaternion)

        quaternion = [x/ norm for x in quaternion]
        rotation = R.from_quat(quaternion)
        euler = rotation.as_euler('xyz', degrees=False)
        yaw_value = euler[2]

        if yaw_value < 0:
            yaw_value += 2*pi

        
        print(f'Angulo: {yaw_value}')

        # Criação e envio da mensagem local_ned
        self.mavlink_connection.mav.vision_position_estimate_send(
        0,
        #int(time.time() * 1e6),  # Timestamp (microseconds since UNIX epoch or system boot)
        x,     # Local X position
        y,     # Local Y position
        z,     # Local Z position
        0,  # Roll angle
        0, # Pitch angle
        yaw_value  # Yaw angle
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
