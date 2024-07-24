from pymavlink import mavutil
import threading
import time
import math
import numpy as np

def connect_vehicle():
    # Target conection
    #connect_string = '127.0.0.1:14550'   # for SITL
    #connect_string = 'tcp:172.16.166.59:5763'   # for SITL
    connect_string = 'tcp:192.168.15.11:5763'   # for SITL
    #connect_string = '/dev/ttyAMA0'             # for RPi

    # Start a connection
    print("Connecting on: ", connect_string)
    vehicle = mavutil.mavlink_connection(connect_string, baud=57600, source_system=1, source_component=2)
    print("Connected!")

    # Wait for the first heartbeat
    vehicle.wait_heartbeat()
    print("Heartbeat from system (system %u component %u)" % (vehicle.target_system, vehicle.target_component))

    return vehicle

def request_all_data_stream(data_rate=1):
    # Cria a mensagem de solicitação de todos os streams de dados
    vehicle.mav.request_data_stream_send(
        target_system       = vehicle.target_system,    # ID do sistema de destino (1 para o próprio veículo)
        target_component    = vehicle.target_component, # ID do componente de destino (0 para todos os componentes)
        req_stream_id       = 0,                        # ID da mensagem (0 para todas)
        req_message_rate    = data_rate,                # Taxa de mensagem (Hz)
        start_stop          = 1                         # Começa ou para o stream (1 para começar)
        )

def request_data_stream():
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

    # Request each msg in the list
    for rqt in rqt_msg:
        # MAVLink command for SET_MESSAGE_INTERVAL
        vehicle.mav.command_long_send(
            vehicle.target_system,      # Target system ID
            vehicle.target_component,   # Target component ID
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

def listener(vehicle):
    # Esta funcao foi criada para rodar em uma thread separada
    while True:
        msg = vehicle.recv_match(type='ATTITUDE', blocking=False)
        if msg:
            vehicle.roll        = msg.roll          # [rad]
            vehicle.pitch       = msg.pitch         # [rad]
            vehicle.yaw         = msg.yaw           # [rad]
            vehicle.rollspeed   = msg.rollspeed     # [rad/s]
            vehicle.pitchspeed  = msg.pitchspeed    # [rad/s]
            vehicle.yawspeed    = msg.yawspeed      # [rad/s]

        msg = vehicle.recv_match(type='VFR_HUD', blocking=False)
        if msg:
            vehicle.airspeed    = msg.airspeed      # [m/s]
            vehicle.groundspeed = msg.groundspeed   # [m/s]
            vehicle.heading     = msg.heading       # [deg]
            vehicle.throttle    = msg.throttle      # [%]
            vehicle.alt         = msg.alt           # [m]   - MSL
            vehicle.climb       = msg.climb         # [m/s] - climb rate    

        msg = vehicle.recv_match(type='GLOBAL_POSITION_INT', blocking=False)
        if msg:
            vehicle.relative_alt= msg.relative_alt*0.001 # [m]
            vehicle.vx          = msg.vx*0.01            # [m/s]
            vehicle.vy          = msg.vy*0.01            # [m/s]
            vehicle.vz          = msg.vz*0.01            # [m/s]
        
        msg = vehicle.recv_match(type='LOCAL_POSITION_NED', blocking=False)
        if msg:
            vehicle.x          = msg.x  # [m]
            vehicle.y          = msg.y  # [m]
            vehicle.z          = msg.z  # [m]

        time.sleep(0.1)

def heartbeat_RPi():
    # Esta funcao foi criada para rodar em uma thread separada
    while True:
        # Send heartbeat from a MAVLink application.
        vehicle.mav.heartbeat_send(type      = mavutil.mavlink.MAV_TYPE_ONBOARD_CONTROLLER,     # Vehicle or component type
                                autopilot    = mavutil.mavlink.MAV_AUTOPILOT_INVALID,           # Autopilot type
                                base_mode    = 0,                                               # System mode bitmap
                                custom_mode  = 0,                                               # System status flag
                                system_status= 0)
        time.sleep(1)

def send_lidar_distances(lidar_scan):
    # Esta funcao foi criada para rodar em uma thread separada
    while True:
        for measure in lidar_scan:
            if measure.angle%45 == 0:
                # Send distances from LD19 Lidar to Ardupilot
                vehicle.mav.distance_sensor_send(
                    time_boot_ms     = 0,                     # Timestamp in ms (time since system boot)
                    min_distance     = 2,                     # cm
                    max_distance     = 1200,                  # cm
                    current_distance = measure.distance,      # cm
                    type             = mavutil.mavlink.MAV_DISTANCE_SENSOR_LASER,
                    id               = 1,                     # Onboard ID of the sensor
                    orientation      = int(measure.angle/45), # Direction the sensor faces (0-7 means YAW 0-45-90-..-360)
                    covariance       = 0                      # Measurement variance (UINT8_MAX if unknown)
                    )
        # Update rate = up to 15Hz
        time.sleep(1/15)    

# Class for msg request configuration
class lidar_scan_class:
    def __init__(self,distance,angle):
        self.distance = distance    # mavlink message ID
        self.angle = angle          # message interval in us

def arm_and_takeoff(target_altitude):

    # Mode = GUIDED
    vehicle.set_mode("GUIDED")
    
    # Arm drone
    vehicle.arducopter_arm()
    print("Armando..")

    # Wait until vehicle armed
    vehicle.motors_armed_wait()
    print("Drone armado")

    # Send TakeOff command message
    vehicle.mav.command_long_send(
        vehicle.target_system,      # ID do sistema de destino
        vehicle.target_component,   # ID do componente de destino
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
    while vehicle.relative_alt < target_altitude*0.95:
        print("Altitude: ", vehicle.relative_alt)
        time.sleep(0.5)

    print("Altitude atingida")

def send_position_target_local_ned(x,y,z=None,
                                   vx=0.0,    vy=0.0, vz=0.0,
                                   afx=0.0,   afy=0.0,afz=0.0,
                                   yaw_angle = None,yaw_rate = 0.0):
    """
    z is negative
    yaw_angle in [degrees]
    yaw_rate in [degrees/s]
    velocities and aceletations ignored
    """
    if z is None:
        # Holds original alt
        z = -vehicle.relative_alt # NED => z is down (negative values)
    
    if yaw_angle is None:
        # Holds original heading (yaw is more precise than heading)
        yaw_angle = math.degrees(vehicle.yaw)

    # Send MAVLink Message
    vehicle.mav.set_position_target_local_ned_send(
        0,                          # time_boot_ms
        vehicle.target_system,      # Target system
        vehicle.target_component,   # Target component
        mavutil.mavlink.MAV_FRAME_LOCAL_NED, # coordinate_frame (with origin fixed relative to earth.)
        0b101111111000,         # Bitmap to indicate which dimensions should be ignored by the vehicle.
        #0b111111111000,         # Ignora YAW => Drone anda sempre de frente (vira na direcao do movimento)
        x,                      # X Position in NED frame [m]
        y,                      # Y Position in NED frame [m]
        z,                      # Z Position in NED frame [m]
        vx,                     # X velocity in NED frame [m/s]
        vy,                     # Y velocity in NED frame [m/s]
        vz,                     # Z velocity in NED frame [m/s]
        afx,                    # X acceleration or force [m/s2]
        afy,                    # Y acceleration or force [m/s2]
        afz,                    # Z acceleration or force [m/s2]
        math.radians(yaw_angle),# Yaw setpoint [rad]
        math.radians(yaw_rate)  # Body yaw rate [rad/s]
    )

def drone_frame_to_local_ned(drone_frame_coord, drone_yaw):
    """
    drone_frame_coord = (x,y,z)
    drone_yaw in radians
    """
    # Class for 3d position
    class pos3:
        def __init__(self,x,y,z):
            self.x = x
            self.y = y
            self.z = z

    rotation_matrix = np.array([
        [np.cos(drone_yaw), -np.sin(drone_yaw), 0],
        [np.sin(drone_yaw), np.cos(drone_yaw), 0],
        [0, 0, 1]
    ])
    local_ned_coordinates = np.dot(rotation_matrix, drone_frame_coord)
    return pos3(local_ned_coordinates[0],local_ned_coordinates[1],local_ned_coordinates[2])

def position_target_wait(target_coord, tolerance, timeout):
    i=0
    while i<timeout:
        if ((abs(target_coord.x-vehicle.x)<=tolerance) and 
        (abs(target_coord.y-vehicle.y)<=tolerance)):
            print("Target position reached!")
            break
        i+=1
        time.sleep(1)

def send_attitude_target(roll_angle = 0.0, pitch_angle = 0.0,
                         yaw_angle = None, yaw_rate = 0.0, use_yaw_rate = False,
                         thrust = 0.5):
    """
    use_yaw_rate: the yaw can be controlled using yaw_angle OR yaw_rate.
                  When one is used, the other is ignored by Ardupilot.
                  
    thrust: 0 <= thrust <= 1, as a fraction of maximum vertical thrust.
            Thrust >  0.5: Ascend
            Thrust == 0.5: Hold the altitude
            Thrust <  0.5: Descend
    """
    if yaw_angle is None:
        # this value may be unused by the vehicle, depending on use_yaw_rate
        msg = vehicle.recv_match(type='ATTITUDE', blocking=True)
        yaw_angle = math.degrees(msg.yaw)
   
    vehicle.mav.set_attitude_target_send(
        0,                          # time_boot_ms
        vehicle.target_system,      # Target system
        vehicle.target_component,   # Target component
        0b00000000 if use_yaw_rate else 0b00000100,
        to_quaternion(roll_angle, pitch_angle, yaw_angle), # Quaternion
        0,                          # Body roll rate in rad/s
        0,                          # Body pitch rate in rad/s
        math.radians(yaw_rate),     # Body yaw rate in rad/s
        thrust                      # Thrust
    )

def set_attitude(roll_angle = 0.0, pitch_angle = 0.0,
                 yaw_angle = None, yaw_rate = 0.0, use_yaw_rate = False,
                 thrust = 0.5, duration = 0):
    """
    Note that from AC3.3 the message should be re-sent more often than every
    second, as an ATTITUDE_TARGET order has a timeout of 1s.
    In AC3.2.1 and earlier the specified attitude persists until it is canceled.
    The code below should work on either version.
    Sending the message multiple times is the recommended way.

    => angles in [degrees]
    => yaw_rate in [degrees/s]
    """
    start = time.time()
    while time.time() - start < duration:
        send_attitude_target(roll_angle, pitch_angle,
                             yaw_angle, yaw_rate, use_yaw_rate,
                             thrust)
        time.sleep(0.1)
    
    # Stop attitude
    while vehicle.groundspeed > 0.15:
        send_attitude_target(-roll_angle, -pitch_angle,
                             yaw_angle, -yaw_rate, use_yaw_rate,
                             thrust)
        time.sleep(0.1)
        

def to_quaternion(roll = 0.0, pitch = 0.0, yaw = 0.0):
    """
    Convert degrees to quaternions
    """
    t0 = math.cos(math.radians(yaw * 0.5))
    t1 = math.sin(math.radians(yaw * 0.5))
    t2 = math.cos(math.radians(roll * 0.5))
    t3 = math.sin(math.radians(roll * 0.5))
    t4 = math.cos(math.radians(pitch * 0.5))
    t5 = math.sin(math.radians(pitch * 0.5))

    w = t0 * t2 * t4 + t1 * t3 * t5
    x = t0 * t3 * t4 - t1 * t2 * t5
    y = t0 * t2 * t5 + t1 * t3 * t4
    z = t1 * t2 * t4 - t0 * t3 * t5

    return [w, x, y, z]

#######################################################################################################################
##                                              CODE STARTS HERE
#######################################################################################################################

# Conexao com o drone
vehicle = connect_vehicle() 

# Cria a thread para enviar HEARTBEAT a cada 1Hz
thread_heartbeat = threading.Thread(target=heartbeat_RPi)
thread_heartbeat.daemon = True  # Define a thread como um daemon para que ela seja encerrada quando o programa principal terminar
thread_heartbeat.start()

# Cria a thread Listener de mensagens MAVLink
thread_listener = threading.Thread(target=listener, args=(vehicle,))
thread_listener.daemon = True  # Define a thread como um daemon para que ela seja encerrada quando o programa principal terminar
thread_listener.start()

# -----------------------------------------------------------
lidar_scan = [] # ONLY FOR TEST
lidar_scan.append(lidar_scan_class(distance=100,angle=0))
lidar_scan.append(lidar_scan_class(distance=200,angle=45))
lidar_scan.append(lidar_scan_class(distance=300,angle=90))
# -----------------------------------------------------------

# # Cria a thread para envio de distancias LIDAR
thread_lidar= threading.Thread(target=send_lidar_distances, args=(lidar_scan,))
thread_lidar.daemon = True  # Define a thread como um daemon para que ela seja encerrada quando o programa principal terminar
thread_lidar.start()

# Solicita as mensagens MAVLink 
request_all_data_stream() # todas as mensagens em 1Hz (para teste)
request_data_stream()
time.sleep(20)

print("vai comecar")
while 1:
    time.sleep(1)

# Decola
arm_and_takeoff(1)
time.sleep(5)

# # Move pela Attitude
# # set_attitude(pitch_angle =  0, roll_angle = 0,yaw_angle=90, yaw_rate=-36,use_yaw_rate=False,duration = 10)

# # Move pela Posicao
# target_position = (50,0,0)                                              # (x,y,z) in drone frame
# target_coord = drone_frame_to_local_ned(target_position, vehicle.yaw)   # (N,E,D) in local ned frame (ajusta heading)
# send_position_target_local_ned(x=target_coord.x, y=target_coord.y)
# position_target_wait(target_coord,tolerance=0.5,timeout=120)

# Mode = LAND
vehicle.set_mode("LAND")

# # while 1:
    # print("Alt =", vehicle.relative_alt)
# #     time.sleep(1)