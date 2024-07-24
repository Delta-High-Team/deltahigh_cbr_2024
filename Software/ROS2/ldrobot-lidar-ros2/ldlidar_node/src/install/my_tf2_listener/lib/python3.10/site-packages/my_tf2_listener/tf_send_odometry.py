#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException
from pymavlink import mavutil

class TF2Listener(Node):
    def __init__(self):
        super().__init__('tf2_listener')
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.timer = self.create_timer(0.1, self.on_timer)
        
        # Configuração da comunicação MAVLink
        self.mavlink_connection = mavutil.mavlink_connection('tcp:127.0.0.1:5762', source_system=1, source_component=1)
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
            self.send_odometry(trans.transform.translation.x,
                               trans.transform.translation.y,
                               trans.transform.translation.z,
                               trans.transform.rotation.w,
                               trans.transform.rotation.z)
            
        except (LookupException, ConnectivityException, ExtrapolationException) as e:
            self.get_logger().warn(f"Could not transform 'map' to 'ldlidar_link': {e}")
    
    def send_odometry(self, x, y, z, w, qz):
        # Criação e envio da mensagem de odometria
        self.mavlink_connection.mav.odometry_send(
            time_usec=int(self.get_clock().now().nanoseconds / 1000),
            frame_id=0,  # FRAME_LOCAL_NED
            child_frame_id=0,  # FRAME_LOCAL_NED
            x=x,
            y=y,
            z=z,
            q=[0, 0, qz, w],  # Quaternions
            vx=0, vy=0, vz=0,  # Velocidades (não usadas)
            rollspeed=0, pitchspeed=0, yawspeed=0,  # Velocidades angulares (não usadas)
            pose_covariance=[0] * 21,  # Covariância de posição (não usada)
            velocity_covariance=[0] * 21,  # Covariância de velocidade (não usada)
            reset_counter=0
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
