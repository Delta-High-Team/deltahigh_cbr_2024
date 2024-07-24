import rclpy
import time
import math
from rclpy.node import Node
from pymavlink import mavutil
from sensor_msgs.msg import LaserScan

class DroneNode(Node):
    def __init__(self):
        super().__init__('drone_node')
        self.create_timer(1, self.update)
        self.create_timer(1, self.send_heartbeat)
        self.create_timer(0.1, self.listener)
        self.create_subscription(LaserScan, '/scan', self.lidar_callback, 10)

        # Configurar a conexão com o drone usando PyMAVLink
        #connect_string = 'tcp:172.16.104.244:5763'   # for SITL
        connect_string = 'tcp:192.168.15.11:5763'   # for SITL
        #connect_string = '/dev/ttyAMA0'             # for RPi
        
        # Start a connection
        print("Connecting on: ", connect_string)
        self.vehicle = mavutil.mavlink_connection(connect_string, baud=57600, 
                                                  source_system=1, source_component=2)
        
        # Wait for the first heartbeat
        self.vehicle.wait_heartbeat()
        print("Heartbeat from system (system %u component %u)" % (self.vehicle.target_system, 
                                                                  self.vehicle.target_component))
        
        # Solicita mensagens MAVLink
        self.request_data_stream()

    def update(self):
        pass

    def send_heartbeat(self):
        # Send heartbeat from Companion Computer
        self.vehicle.mav.heartbeat_send(type=mavutil.mavlink.MAV_TYPE_ONBOARD_CONTROLLER,     # Vehicle or component type
                                        autopilot    = mavutil.mavlink.MAV_AUTOPILOT_INVALID, # Autopilot type
                                        base_mode    = 0,                                     # System mode bitmap
                                        custom_mode  = 0,                                     # System status flag
                                        system_status= 0)

    def listener(self):
        msg = self.vehicle.recv_match(type='ATTITUDE', blocking=False)
        if msg:
            self.vehicle.roll        = msg.roll          # [rad]
            self.vehicle.pitch       = msg.pitch         # [rad]
            self.vehicle.yaw         = msg.yaw           # [rad]
            self.vehicle.rollspeed   = msg.rollspeed     # [rad/s]
            self.vehicle.pitchspeed  = msg.pitchspeed    # [rad/s]
            self.vehicle.yawspeed    = msg.yawspeed      # [rad/s]

        msg = self.vehicle.recv_match(type='VFR_HUD', blocking=False)
        if msg:
            self.vehicle.airspeed    = msg.airspeed      # [m/s]
            self.vehicle.groundspeed = msg.groundspeed   # [m/s]
            self.vehicle.heading     = msg.heading       # [deg]
            self.vehicle.throttle    = msg.throttle      # [%]
            self.vehicle.alt         = msg.alt           # [m]   - MSL
            self.vehicle.climb       = msg.climb         # [m/s] - climb rate    

        msg = self.vehicle.recv_match(type='GLOBAL_POSITION_INT', blocking=False)
        if msg:
            self.vehicle.relative_alt= msg.relative_alt*0.001 # [m]
            self.vehicle.vx          = msg.vx*0.01            # [m/s]
            self.vehicle.vy          = msg.vy*0.01            # [m/s]
            self.vehicle.vz          = msg.vz*0.01            # [m/s]
        
        msg = self.vehicle.recv_match(type='LOCAL_POSITION_NED', blocking=False)
        if msg:
            self.vehicle.x          = msg.x  # [m]
            self.vehicle.y          = msg.y  # [m]
            self.vehicle.z          = msg.z  # [m]

        time.sleep(0.1)
        
    def request_data_stream(self):
        # Class for msg request configuration
        class rqt_msg_config:
            def __init__(self,msg_id,msg_interval):
                self.msg_id = msg_id                # mavlink message ID
                self.msg_interval = msg_interval    # message interval in us
        
        # List of MAVLink messages request
        rqt_msg = []
        rqt_msg.append(rqt_msg_config(msg_id = mavutil.mavlink.MAVLINK_MSG_ID_VFR_HUD,  msg_interval = 1e6/4))
        rqt_msg.append(rqt_msg_config(msg_id = mavutil.mavlink.MAVLINK_MSG_ID_ATTITUDE, msg_interval = 1e6/4))
        rqt_msg.append(rqt_msg_config(msg_id = mavutil.mavlink.MAVLINK_MSG_ID_GLOBAL_POSITION_INT,msg_interval = 1e6/2))
        rqt_msg.append(rqt_msg_config(msg_id = mavutil.mavlink.MAVLINK_MSG_ID_LOCAL_POSITION_NED,msg_interval = 1e6/2))

        print("Requesting data stream..")

        # Request each msg in the list
        for rqt in rqt_msg:
            # MAVLink command for SET_MESSAGE_INTERVAL
            self.vehicle.mav.command_long_send(
                self.vehicle.target_system,      # Target system ID
                self.vehicle.target_component,   # Target component ID
                mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,  # ID of command to send
                0,                          # Confirmation
                rqt.msg_id,                 # param1: Message ID to be streamed
                rqt.msg_interval,           # param2: Interval in microseconds
                0,                          # param3 (unused)
                0,                          # param4 (unused)
                0,                          # param5 (unused)
                0,                          # param5 (unused)
                0                           # param6 (unused)
            )
        
    def lidar_callback(self,topic_msg):
        print("Lidar callback")
        angles = []
        distances = []

        # Cálculo dos ângulos para cada ponto da medição do lidar
        angle_min = topic_msg.angle_min
        angle_increment = topic_msg.angle_increment
        for i, range_data in enumerate(topic_msg.ranges):
            angle = angle_min + i * angle_increment
            angles.append(angle)
            distances.append(range_data)

        # Conversão dos ângulos de radianos para graus
        angles_degrees = [math.degrees(angle) for angle in angles]

        # Construção de uma lista de pares (ângulo, distância)
        data = list(zip(angles_degrees, distances))
        
        # Envio da mensagem MAVLink para cada par (ângulo, distância)
        for angle, distance in data:
            # Conversao ROS frame to NED frame (roll 180)
            angle = abs(angle-360)

            if round(angle)%45 == 0 and round(angle) in range(360) and not math.isinf(distance):
                # Send distances from LD19 Lidar to Ardupilot
                self.vehicle.mav.distance_sensor_send(
                    time_boot_ms     = 0,                     # Timestamp in ms (time since system boot)
                    min_distance     = 2,                     # cm
                    max_distance     = 1200,                  # cm
                    current_distance = round(distance*100),   # cm
                    type             = mavutil.mavlink.MAV_DISTANCE_SENSOR_LASER,
                    id               = 0,                     # Onboard ID of the sensor
                    orientation      = int(round(angle)/45),  # Direction the sensor faces (0-7 means YAW 0-45-90-135-180-225-270-315)
                    covariance       = 0                      # Measurement variance (UINT8_MAX if unknown)
                    )

    def arm_and_takeoff(self, target_altitude):

        # Mode = GUIDED
        self.vehicle.set_mode("GUIDED")
        
        # Arm drone
        self.vehicle.arducopter_arm()
        print("Armando..")

        # Wait until vehicle armed
        self.vehicle.motors_armed_wait()
        print("Drone armado")

        # Send TakeOff command message
        self.vehicle.mav.command_long_send(
            self.vehicle.target_system,      # ID do sistema de destino
            self.vehicle.target_component,   # ID do componente de destino
            mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,  # Comando de decolagem
            0,                          # Confirmation
            0,                          # Param 1 (Desconsiderado)
            0,                          # Param 2 (Desconsiderado)
            0,                          # Param 3 (Desconsiderado)
            0,                          # Param 4 (Desconsiderado)
            0,                          # Param 5 (Latitude, Desconsiderado)
            0,                          # Param 6 (Longitude, Desconsiderado)
            target_altitude             # Param 7 (Altura de decolagem em metros)
        )

        # Wait until reach target altitude
        while self.vehicle.relative_alt < target_altitude*0.95:
            print("Altitude: ", self.vehicle.relative_alt)
            time.sleep(0.5)

        print("Altitude atingida")

def main(args=None):
    rclpy.init(args=args)
    node = DroneNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
