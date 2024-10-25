from quart import Quart, jsonify, request, Response
from quart_cors import cors
from pymavlink import mavutil
import serial.tools.list_ports
import asyncio
import time
from pymavlink.dialects.v20 import ardupilotmega as mavlink2
from hypercorn.asyncio import serve
from hypercorn.config import Config
import logging
import math
import platform
import json
from collections import deque, defaultdict

app = Quart(__name__)
app = cors(app, 
    allow_origin=["http://localhost:5173", "http://localhost:3000"],
    allow_credentials=True,
    allow_methods=["GET", "POST", "PUT", "DELETE", "OPTIONS"],
    allow_headers=["Content-Type"]
)

last_heartbeat_time = 0
packet_count = 0
total_packets = 0
connection_quality = 0
total_params = 0
parameters = {}
missing_parameters = set()
param_count = 0
param_fetch_start = 0
MAX_RETRIES = 3
PARAM_TIMEOUT = 0.3  # 300ms timeout for individual parameter
FETCH_TIMEOUT = 30
fetch_complete = asyncio.Event()
home_position_set = False
home_coordinates = {"lat": None, "lon": None, "alt": None}
connection = None
mav = None
global_device = None
global_baudrate = None
global_protocol = None

drone_status = {
    "connection_status": "Disconnected","system_id": None,"component_id": None,"firmware_type": None,"vehicle_type": None,"battery_status": {},"gps": None,"satellite_count": None,"hdop": None,"gps_fix_type": None,
    "armed": False,"firmware_version": None,"board_version": None,"rssi_dBm": None,"remrssi_dBm": None,"channel_outputs": {},"distance_to_home": None,"rangefinder": {},"rx_errors": None,"tx_buffer": None,
    "local_noise": None,"remote_noise": None,"attitude": {"pitch": None,"yaw": None,"roll": None},"groundspeed": None,"airspeed": None,"heading": None,"mav_mode": None,"mav_landed_state": None 
}

logging.basicConfig(level=logging.DEBUG, format='%(asctime)s - %(levelname)s - %(message)s')
# logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')
logger = logging.getLogger(__name__)

MAX_AP_MESSAGES = 1000 
ap_messages = deque(maxlen=MAX_AP_MESSAGES)
ap_messages_lock = asyncio.Lock()

calibration_data = {
    "status_text": "",
    "heartbeat": "Disconnected",
    "compasses": [],
    "calibration_started": False
}

ACCEL_CAL_POSITIONS = {
    "level": 1,
    "left": 2,
    "right": 3,
    "nose_up": 4,
    "nose_down": 5,
    "tail_down": 6
}

accel_calibration_status = {
    "in_progress": False,
    "type": None,
    "message": "",
    "completed": False,
    "connected": False,
    "last_command_result": None,
    "system_id": None,
    "component_id": None,
    "version": None,
    "vehicle_type": None,
}

rc_values = {
    "channels": {},
    "last_update": 0
}

radio_calibration_status = {
    "is_calibrating": False,
    "current_step": None,
    "channel_count": 0,
    "calibration_data": {},
    "status_message": "",
    "last_command_ack": None,
    "saving_parameters": False
}






class CommandQueue:
    def __init__(self):
        self.queue = asyncio.Queue()
        self.waiting_ack = {}
        self.retry_counts = {} #from radio cali
        self.max_retries = 3 #from radio cali

    async def add_command(self, command, *args):
        await self.queue.put((command, args))
        if command not in self.retry_counts:
            self.retry_counts[command] = 0

    async def process_queue(self):
        while True:
            if not connection or not connection.target_system:
                await asyncio.sleep(1)
                continue
            try:
                command, args = await self.queue.get()
                args = list(args) + [0] * (7 - len(args))

                command_id = connection.mav.command_long_encode(
                    connection.target_system, connection.target_component,
                    command, 0, *args
                ).command

                self.waiting_ack[command_id] = asyncio.get_event_loop().time()
                self.waiting_ack[command] = { #
                    'timestamp': asyncio.get_event_loop().time(), # from radio calibration
                    'args': args #
                } #

                connection.mav.command_long_send(
                    connection.target_system, connection.target_component,
                    command, 0, *args
                )
                logger.info(f"Sent command: {command} with args: {args}")
            except Exception as e:
                logger.error(f"Error processing command: {e}")
            await asyncio.sleep(0.1)

    def ack_command(self, command_id):
        if command_id in self.waiting_ack:
            del self.waiting_ack[command_id]
            if command_id in self.retry_counts:
                del self.retry_counts[command_id]

    async def retry_commands(self):
        while True:
            current_time = asyncio.get_event_loop().time()

            for command_id, command_data, send_time in list(self.waiting_ack.items()):
                if current_time - send_time > 3:
                    if self.retry_counts.get(command_id, 0) < self.max_retries:
                        logger.warning(f"Command {command_id} timed out, retry {self.retry_counts[command_id] + 1}/{self.max_retries}")
                        await self.add_command(command_id, *command_data['args'])
                        self.retry_counts[command_id] = self.retry_counts.get(command_id, 0) + 1
                    else:
                        logger.error(f"Command {command_id} failed after {self.max_retries} retries")
                        del self.waiting_ack[command_id]
                        del self.retry_counts[command_id]
            await asyncio.sleep(1)

command_queue = CommandQueue()

MAV_RESULT_MAP = {
    mavutil.mavlink.MAV_RESULT_ACCEPTED: "ACCEPTED",
    mavutil.mavlink.MAV_RESULT_TEMPORARILY_REJECTED: "TEMPORARILY_REJECTED",
    mavutil.mavlink.MAV_RESULT_DENIED: "DENIED",
    mavutil.mavlink.MAV_RESULT_UNSUPPORTED: "UNSUPPORTED",
    mavutil.mavlink.MAV_RESULT_FAILED: "FAILED",
    mavutil.mavlink.MAV_RESULT_IN_PROGRESS: "IN_PROGRESS",
}

MAV_CMD_MAP = {
    mavutil.mavlink.MAV_CMD_PREFLIGHT_CALIBRATION: "PREFLIGHT_CALIBRATION",
    mavutil.mavlink.MAV_CMD_ACCELCAL_VEHICLE_POS: "ACCELCAL_VEHICLE_POS",
    mavutil.mavlink.MAV_CMD_DO_SEND_BANNER: "DO_SEND_BANNER",
    mavutil.mavlink.MAV_CMD_REQUEST_AUTOPILOT_CAPABILITIES: "REQUEST_AUTOPILOT_CAPABILITIES",
}





async def get_parameter(param_name):
    connection.mav.param_request_read_send(
        connection.target_system, connection.target_component,
        param_name.encode(), -1
    )
    start_time = time.time()
    while time.time() - start_time < 1:
        msg = connection.recv_match(type='PARAM_VALUE', blocking=False)
        if msg and msg.param_id == param_name:
            return msg.param_value
        await asyncio.sleep(0.1)
    logger.warning(f"Timeout getting parameter: {param_name}")
    return None

async def request_autopilot_capabilities():
    if connection and mav:
        mav.command_long_send(
            connection.target_system, connection.target_component,
            mavutil.mavlink.MAV_CMD_REQUEST_AUTOPILOT_CAPABILITIES,
            0, 1, 0, 0, 0, 0, 0, 0
        )
        logger.info("Requested autopilot capabilities")

async def send_banner_request():
    if connection and mav:
        mav.command_long_send(
            connection.target_system, connection.target_component,
            mavutil.mavlink.MAV_CMD_DO_SEND_BANNER,
            0, 0, 0, 0, 0, 0, 0, 0
        )
        logger.info("Sent banner request")

# async def send_banner_request():
#     await command_queue.add_command(
#         mavutil.mavlink.MAV_CMD_DO_SEND_BANNER,
#         0, 0, 0, 0, 0, 0, 0
#     )



async def request_data_streams():
    """Request essential data streams from the drone"""
    if not connection or not connection.target_system:
        return
    
    streams = {
        mavutil.mavlink.MAV_DATA_STREAM_EXTENDED_STATUS: 2,
        mavutil.mavlink.MAV_DATA_STREAM_POSITION: 2,
        mavutil.mavlink.MAV_DATA_STREAM_EXTRA1: 4,
        mavutil.mavlink.MAV_DATA_STREAM_EXTRA2: 4,
        mavutil.mavlink.MAV_DATA_STREAM_EXTRA3: 2,
        mavutil.mavlink.MAV_DATA_STREAM_RAW_SENSORS: 2,
        mavutil.mavlink.MAV_DATA_STREAM_RC_CHANNELS: 2,
    }

    for stream_id, rate in streams.items():
        connection.mav.request_data_stream_send(
            connection.target_system,
            connection.target_component,
            stream_id,
            rate,
            1  # Start sending
        )
    logger.info("Data streams requested")
    
# async def request_autopilot_version():
#     await command_queue.add_command(
#         mavutil.mavlink.MAV_CMD_REQUEST_AUTOPILOT_CAPABILITIES,
#         1, 0, 0, 0, 0, 0, 0
#     )





async def detect_compasses():
    global calibration_data
    calibration_data["compasses"] = []
    
    compass_params = ['COMPASS_USE', 'COMPASS_USE2', 'COMPASS_USE3']
    
    for i, param in enumerate(compass_params):
        use_compass = await get_parameter(param)
        if use_compass is not None and use_compass == 1:
            calibration_data["compasses"].append({
                "id": i,
                "name": f"Compass {i+1}",
                "enabled": True,
                "progress": 0,
                "calibrated": False,
                "report": None
            })
            logger.info(f"Detected enabled compass: Compass {i+1}")
        else:
            logger.info(f"Compass {i+1} is disabled or not detected")

    if not calibration_data["compasses"]:
        logger.warning("No enabled compasses detected")
    else:
        logger.info(f"Detected compasses: {calibration_data['compasses']}")

async def connect_to_drone(connection_string, baudrate=115200):
    global connection, mav, global_device, global_baudrate, global_protocol, last_heartbeat_time
    try:
        connection = mavutil.mavlink_connection(connection_string, baud=baudrate)
        mav = mavutil.mavlink.MAVLink(connection, srcSystem=255, srcComponent=0)
        await asyncio.to_thread(connection.wait_heartbeat)
        logger.info("Heartbeat received. Drone is online.")
        await request_data_streams()
        return True
    except Exception as e:
        logger.error(f"Connection failed: {str(e)}")
        return False

#combined code
@app.route('/connect', methods=['POST'])
async def connect_drone():
    global connection, home_position_set, mav, global_device, global_baudrate, global_protocol, last_heartbeat_time, accel_calibration_status
    data = await request.get_json()
    protocol = data.get("protocol")
    
    if protocol == "serial":
        device = data.get("port")
        baudrate = data.get("baudrate", 115200)
    elif protocol in ["udp", "tcp"]:
        ip = data.get("ip")
        port = data.get("port")
        device = f"{protocol}:{ip}:{port}"
    else:
        return jsonify({"status": "Invalid connection type"}), 400
    while True:
        try:
            global_device = device
            global_baudrate = baudrate if protocol == 'serial' else None
            global_protocol = protocol
            connection = mavutil.mavlink_connection(
                device, 
                baud=global_baudrate, 
                source_system=255, 
                source_component=0, 
                autoreconnect=True, 
                timeout=60
            )
            mav = mavlink2.MAVLink(connection)
            mav.srcSystem = 255
            mav.srcComponent = 0
            msg = await asyncio.to_thread(connection.wait_heartbeat, timeout=10)
            if msg:
                logger.info(f"Heartbeat from system (system {connection.target_system} component {connection.target_component})")
                last_heartbeat_time = time.time()
                home_position_set = False

                accel_calibration_status["system_id"] = connection.target_system
                accel_calibration_status["component_id"] = connection.target_component
                accel_calibration_status["connected"] = True
                accel_calibration_status["message"] = "Connected to drone."
                accel_calibration_status["vehicle_type"] = msg.type

                await request_data_streams()
                # await request_autopilot_version()
                await request_autopilot_capabilities()
                await send_banner_request()
                await detect_compasses()

                asyncio.create_task(fetch_drone_data())
                asyncio.create_task(send_heartbeat())

                return jsonify({"status": f"Connected to drone via {protocol.upper()}"}), 200
            else:
                accel_calibration_status["connected"] = False
                accel_calibration_status["message"] = "No heartbeat received"

                # logger.error("No heartbeat received")
                return jsonify({"status": "No heartbeat received"}), 500
        except Exception as e:
            logger.error(f"Error connecting to drone: {e}")
            accel_calibration_status["connected"] = False
            accel_calibration_status["message"] = f"Failed to connect: {e}"

            # return jsonify({"status": f"Failed to connect: {str(e)}"}), 500
        await asyncio.sleep(5)

#existing code 
# @app.route('/connect', methods=['POST'])
# async def connect_drone():
#     global connection, home_position_set, mav, global_device, global_baudrate, global_protocol, last_heartbeat_time, calibration_status
#     data = await request.get_json()
#     protocol = data.get("protocol")
    
#     if protocol == "serial":
#         device = data.get("port")
#         baudrate = data.get("baudrate", 115200)
#     elif protocol in ["udp", "tcp"]:
#         ip = data.get("ip")
#         port = data.get("port")
#         device = f"{protocol}:{ip}:{port}"
#     else:
#         return jsonify({"status": "Invalid connection type"}), 400
#     try:
#         # NEW: Store connection details globally
#         global_device = device
#         global_baudrate = baudrate if protocol == 'serial' else None
#         global_protocol = protocol
#         connection = mavutil.mavlink_connection(device, baud=global_baudrate, source_system=255, source_component=0, autoreconnect=True, timeout=60)
#         mav = mavlink2.MAVLink(connection)
#         mav.srcSystem = 255
#         mav.srcComponent = 0
#         await asyncio.to_thread(connection.wait_heartbeat, timeout=10)
#         last_heartbeat_time = time.time()
#         home_position_set = False
#         await request_data_streams()
#         await request_autopilot_capabilities()
#         await send_banner_request()
#         await fetch_parameter_file()
#         await detect_compasses()
#         asyncio.create_task(fetch_drone_data())
#         asyncio.create_task(send_heartbeat())
#         return jsonify({"status": f"Connected to drone via {protocol.upper()}"}), 200
#     except Exception as e:
#         return jsonify({"status": f"Failed to connect: {str(e)}"}), 500

async def send_heartbeat():
    while True:
        if connection and mav:
            try:
                mav.heartbeat_send(
                    mavutil.mavlink.MAV_TYPE_GCS,
                    mavutil.mavlink.MAV_TYPE_ONBOARD_CONTROLLER,
                    mavutil.mavlink.MAV_AUTOPILOT_INVALID,
                    0, 0, 0
                )
            except Exception as e:
                logger.error(f"Error sending heartbeat: {e}")
        await asyncio.sleep(1)

async def heartbeat_monitor():
    global accel_calibration_status
    while True:
        if connection is not None:
            try:
                msg = await asyncio.to_thread(connection.recv_match, type='HEARTBEAT', blocking=True, timeout=5)
                if msg:
                    if not accel_calibration_status["connected"]:
                        accel_calibration_status["connected"] = True
                        accel_calibration_status["message"] = "Reconnected to drone."
                        logger.info("Reconnected to drone.")
                else:
                    if accel_calibration_status["connected"]:
                        accel_calibration_status["connected"] = False
                        accel_calibration_status["message"] = "Lost connection to drone."
                        logger.warning("Lost connection to drone.")
                    await connect_drone()
            except Exception as e:
                logger.error(f"Heartbeat error: {e}")
                accel_calibration_status["connected"] = False
                accel_calibration_status["message"] = f"Heartbeat error: {e}"
                await connect_drone()
        await asyncio.sleep(1)

@app.get("/connection_status")
async def get_connection_status():
    if connection:
        return {"status": "Connected", "last_heartbeat": time.time() - last_heartbeat_time}
    else:
        return {"status": "Disconnected"}
    
async def periodic_connection_check():
    while True:
        if connection:
            if time.time() - last_heartbeat_time > 5:
                logger.warning("No heartbeat received for 5 seconds. Attempting reconnection.")
                await reconnect_drone()
        await asyncio.sleep(1)

async def reconnect_drone():
    global connection, mav, last_heartbeat_time
    if connection:
        connection.close()
    try:
        connection = mavutil.mavlink_connection(global_device, baud=global_baudrate, source_system=255, source_component=0, autoreconnect=True, timeout=60)
        mav = mavutil.mavlink.MAVLink(connection)
        mav.srcSystem = 255
        mav.srcComponent = 0
        await asyncio.to_thread(connection.wait_heartbeat, timeout=10)
        last_heartbeat_time = time.time()
        await request_data_streams()
        logger.info("Reconnected to drone successfully")
    except Exception as e:
        logger.error(f"Failed to reconnect: {str(e)}")

@app.route('/disconnect', methods=['POST'])
async def disconnect_drone():
    global connection
    if connection:
        connection.close()
        connection = None
        return jsonify({"status": "Disconnected from drone"}), 200
    else:
        return jsonify({"status": "No active connection"}), 400





def list_serial_ports():
    if platform.system() == "Windows":
        ports = serial.tools.list_ports.comports()
        return [port.device for port in ports]
    else:  # Linux (Ubuntu)
        import glob
        return glob.glob('/dev/tty[A-Za-z]*')

@app.route('/com_ports', methods=['GET'])
async def get_com_ports():
    ports = list_serial_ports()
    return jsonify({"ports": ports}), 200





def reset_telemetry_values():
    global drone_status, packet_count, total_packets, connection_quality
    drone_status.update({
    "connection_status": "Disconnected",
    "system_id": None,"component_id": None,"firmware_type": None,"vehicle_type": None,"battery_status": {},"gps": None,"satellite_count": None,"hdop": None,"gps_fix_type": None,"armed": False,
    "firmware_version": None,"board_version": None,"rssi_dBm": None,"remrssi_dBm": None,"channel_outputs": {},"distance_to_home": None,"rangefinder": {},"rx_errors": None,"tx_buffer": None,"local_noise": None,
    "remote_noise": None,"attitude": {"pitch": None,"yaw": None,"roll": None},"groundspeed": None,"airspeed": None,"heading": None,"mav_mode": None,"mav_landed_state": None
    })
    packet_count = 0
    total_packets = 0
    connection_quality = 0

async def fetch_drone_data():
    global connection, drone_status, last_heartbeat_time, packet_count, total_packets, connection_quality
    heartbeat_timeout = 3
    while True:
        if connection:
            try:
                msg = await asyncio.to_thread(connection.recv_msg)
                if msg:
                    total_packets += 1
                    if msg.get_type() != 'BAD_DATA':
                        packet_count += 1
                        await update_drone_status(msg)
                        await process_mavlink_message(msg)
                        update_calibration_data(msg)
                        if msg.get_type() == 'HEARTBEAT':
                            last_heartbeat_time = time.time()
                    connection_quality = (packet_count / total_packets) * 100 if total_packets > 0 else 0
                if time.time() - last_heartbeat_time > heartbeat_timeout:
                    logger.warning("Heartbeat lost. Clearing drone status.")
                    # reset_telemetry_values()
                    await reconnect_drone()
            except Exception as e:
                logger.error(f"Error receiving data: {e}")
                drone_status["connection_status"] = "Error"
                # await reconnect_drone()
                reset_telemetry_values()
        else:
            logger.warning("No active connection")
            reset_telemetry_values()
            await asyncio.sleep(1)

@app.route('/status', methods=['GET'])
async def get_status():
    global drone_status, connection_quality
    drone_status["connection_quality"] = connection_quality
    return jsonify(drone_status), 200

async def calculate_distance_to_home():
    home_position = await get_home_position()
    if not home_position or 'gps' not in drone_status:
        return None
    
    R = 6371000  # Earth radius in meters
    lat1, lon1 = math.radians(home_position['latitude']), math.radians(home_position['longitude'])
    lat2, lon2 = math.radians(drone_status['gps']['lat']), math.radians(drone_status['gps']['lon'])
    
    dlat = lat2 - lat1
    dlon = lon2 - lon1
    a = math.sin(dlat/2)**2 + math.cos(lat1) * math.cos(lat2) * math.sin(dlon/2)**2
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1-a))
    
    distance = R * c
    return distance

def get_command_and_params(wp):
    wp_type = wp['type']
    params = wp['params']

    if wp_type == 'WAYPOINT':
        return (mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, params['Delay'], params['Acceptance Radius'], params['Pass Radius'], params['Yaw Angle'])
    elif wp_type == 'LOITER_UNLIMITED':
        return (mavutil.mavlink.MAV_CMD_NAV_LOITER_UNLIM, params['Radius'], 0, params['Direction'], params['Yaw'])
    elif wp_type == 'LOITER_TIME':
        return (mavutil.mavlink.MAV_CMD_NAV_LOITER_TIME, params['Time'], 0, params['Radius'], params['Yaw'])
    elif wp_type == 'LOITER_TURNS':
        return (mavutil.mavlink.MAV_CMD_NAV_LOITER_TURNS, params['Turns'], 0, params['Radius'], params['Yaw'])
    elif wp_type == 'RETURN_TO_LAUNCH':
        return (mavutil.mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH, 0, 0, 0, 0)
    elif wp_type == 'LAND':
        return (mavutil.mavlink.MAV_CMD_NAV_LAND, params['Abort Alt'], params['Precision Land'], 0, 0)
    elif wp_type == 'TAKEOFF':
        return (mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, params['Pitch Angle'], 0, 0, params['Yaw Angle'])
    elif wp_type == 'DO_CHANGE_SPEED':
        return (mavutil.mavlink.MAV_CMD_DO_CHANGE_SPEED, params['Speed Type'], params['Speed'], params['Throttle'], 0)
    elif wp_type == 'DO_SET_CAM_TRIGG_DIST':
        return (mavutil.mavlink.MAV_CMD_DO_SET_CAM_TRIGG_DIST, params['Distance'], 0, 0, 0, params['Shutter'], params['Trigger'], 0)
    elif wp_type == 'DO_SET_ROI':
        return (mavutil.mavlink.MAV_CMD_DO_SET_ROI, params['ROI Mode'], params['WP Index'], params['ROI Index'], 0, 0, 0, 0)
    else:
        logger.warning(f"Unknown waypoint type: {wp_type}")
        return (mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0)
    
async def set_mode(mode):
    if connection and mav:
        if mode not in connection.mode_mapping():
            print(f'Unknown mode : {mode}')
            print(f'Try:', list(connection.mode_mapping().keys()))
            return False
        mode_id = connection.mode_mapping()[mode]
        return await send_command_long(
            mavutil.mavlink.MAV_CMD_DO_SET_MODE, 
            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            mode_id
        )
    return False

async def get_home_position():
    if not connection:
        return None
    connection.mav.command_long_send(
        connection.target_system,
        connection.target_component,
        mavutil.mavlink.MAV_CMD_GET_HOME_POSITION,
        0,  
        0, 0, 0, 0, 0, 0, 0 
    )
    msg = await asyncio.to_thread(connection.recv_match, type='HOME_POSITION', blocking=True, timeout=5)
    if msg:
        return {
            'latitude': msg.latitude / 1e7,
            'longitude': msg.longitude / 1e7,
            'altitude': msg.altitude / 1000  # convert mm to meters
        }
    return None

@app.route('/get_home_position', methods=['GET'])
async def get_home_position_route():
    if connection is None or connection.mav is None:
        return jsonify({'error': "'NoneType' object has no attribute 'mav'"}), 500
    try:
        msg = await asyncio.to_thread(connection.recv_match, type='HOME_POSITION', blocking=True, timeout=5)
        if msg is None:
            return jsonify({'error': 'Failed to get home position from drone'}), 500
        home_latitude = msg.latitude / 1e7
        home_longitude = msg.longitude / 1e7
        home_altitude = msg.altitude / 1000
        return jsonify({'latitude': home_latitude, 'longitude': home_longitude, 'altitude': home_altitude})
    except Exception as e:
        print(f"Error fetching home position: {e}")
        return jsonify({'error': str(e)}), 500

@app.route('/command', methods=['POST'])
async def command():
    data = await request.get_json()
    command = data['command']
    if command == 'arm':
        success = await send_command_long(mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 1)
    elif command == 'disarm':
        success = await send_command_long(mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0)
    elif command == 'takeoff':
        altitude = float(data['altitude'])
        success = await send_command_long(mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, altitude)
    elif command == 'land':
        success = await send_command_long(mavutil.mavlink.MAV_CMD_NAV_LAND)
    elif command == 'rtl':
        success = await set_mode('RTL')
    elif command == 'set_home':
        latitude = float(data['latitude'])
        longitude = float(data['longitude'])
        # altitude = float(data['altitude'])
        current_alt = drone_status['gps']['relative_alt'] if drone_status['gps'] else 0
        success = await send_command_long(mavutil.mavlink.MAV_CMD_DO_SET_HOME, 0, 0, 0, 0, latitude, longitude,0)
    elif command == 'set_mode':
        mode = data['mode']
        success = await set_mode(mode)
    else:
        return jsonify({'status': 'unknown_command'}), 400
    
    return jsonify({'status': 'success' if success else 'failed'}), 200 if success else 500

@app.route('/radiostatus', methods=['GET'])
async def get_radio_status():
    try:
        return jsonify(accel_calibration_status)
    except Exception as e:
        logger.error(f"Error in status endpoint: {e}")
        return await jsonify({"error": str(e)}, 500)

@app.route('/rc_channels', methods=['GET'])
async def get_rc_channels():
    try:
        return jsonify(rc_values)
    except Exception as e:
        logger.error(f"Error in rc_channels endpoint: {e}")
        return jsonify({"error": str(e)}), 500

@app.route('/parameters', methods=['GET'])
async def get_radio_parameters():
    try:
        # Filter for RC-related parameters
        rc_params = {k: v for k, v in parameters.items() if 'RC' in k}
        return jsonify(rc_params)
    except Exception as e:
        logger.error(f"Error in parameters endpoint: {e}")
        return jsonify({"error": str(e)}), 500





def decode_firmware_version(version):
    major = (version >> 24) & 0xFF
    minor = (version >> 16) & 0xFF
    patch = (version >> 8) & 0xFF
    return f"{major}.{minor}.{patch}"

async def update_drone_status(msg):
    global drone_status, home_position_set, home_coordinates
    msg_type = msg.get_type()

    if msg_type == 'HEARTBEAT':
        drone_status["connection_status"] = "Connected"
        drone_status["system_id"] = connection.target_system
        drone_status["component_id"] = connection.target_component
        drone_status["firmware_type"] = mavutil.mavlink.enums['MAV_AUTOPILOT'][msg.autopilot].description
        drone_status["vehicle_type"] = mavutil.mavlink.enums['MAV_TYPE'][msg.type].description
        drone_status["armed"] = msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED
        drone_status["mav_mode"] = msg.custom_mode
    elif msg_type == 'EXTENDED_SYS_STATE':
        drone_status["mav_landed_state"] = mavutil.mavlink.enums['MAV_LANDED_STATE'][msg.landed_state].description

    elif msg_type == 'RADIO_STATUS':
        drone_status['rssi_dBm'] = (msg.rssi / 1.9) - 127
        drone_status['remrssi_dBm'] = (msg.remrssi / 1.9) - 127
        drone_status['rx_errors'] = msg.rxerrors
        drone_status['tx_buffer'] = msg.txbuf
        drone_status['local_noise'] = msg.noise
        drone_status['remote_noise'] = msg.remnoise

    elif msg_type == 'BATTERY_STATUS':
        drone_status['battery_status'] = {
            "voltage": msg.voltages[0] / 1000.0,"current": msg.current_battery / 100.0,"battery_remaining": msg.battery_remaining,"capacity_consumed": msg.current_consumed
        }

    elif msg_type == 'GPS_RAW_INT':
        drone_status['satellite_count'] = msg.satellites_visible
        drone_status['hdop'] = msg.eph / 100.0
        drone_status['gps_fix_type'] = msg.fix_type

    elif msg_type == 'GLOBAL_POSITION_INT':
        drone_status['gps'] = {
            "lat": msg.lat / 1e7,"lon": msg.lon / 1e7,"alt": msg.alt / 1000.0,"relative_alt": msg.relative_alt / 1000.0
        }

        if not home_position_set:
            home_coordinates['lat'] = drone_status['gps']['lat']
            home_coordinates['lon'] = drone_status['gps']['lon']
            home_coordinates['alt'] = drone_status['gps']['alt']
            home_position_set = True

        vx = msg.vx / 100.0
        vy = msg.vy / 100.0
        vz = msg.vz / 100.0
        drone_status['groundspeed'] = math.sqrt(vx**2 + vy**2)
        drone_status['airspeed'] = math.sqrt(vx**2 + vy**2 + vz**2)
    elif msg_type == 'ATTITUDE':
        drone_status['attitude'] = {
            "roll": math.degrees(msg.roll),
            "pitch": math.degrees(msg.pitch),
            "yaw": math.degrees(msg.yaw)
        }
    elif msg_type == 'VFR_HUD':
        drone_status['heading'] = msg.heading

    elif msg_type == 'DISTANCE_SENSOR':
        drone_status['rangefinder'] = {
            "distance": msg.current_distance / 100.0,"min_distance": msg.min_distance / 100.0,"max_distance": msg.max_distance / 100.0,
        }

    elif msg_type == 'AUTOPILOT_VERSION':
        drone_status['firmware_version'] = decode_firmware_version(msg.flight_sw_version)
        drone_status['board_version'] = msg.board_version

    elif msg_type == 'HOME_POSITION':
        drone_status['distance_to_home'] = await calculate_distance_to_home()

    elif msg_type == 'SERVO_OUTPUT_RAW':
        drone_status['channel_outputs'] = {f"ch{i+1}_output": getattr(msg, f'servo{i+1}_raw') for i in range(8)}
    elif msg_type == 'PARAM_VALUE':  # Handle PARAM_VALUE messages
        parameters[msg.param_id] = msg.param_value





async def send_command_long(command, param1=0, param2=0, param3=0, param4=0, param5=0, param6=0, param7=0):
    if connection and mav:
        mav.command_long_send(
            connection.target_system, connection.target_component,
            command, 0, param1, param2, param3, param4, param5, param6, param7
        )
        ack = await asyncio.to_thread(connection.recv_match, type='COMMAND_ACK', blocking=True, timeout=5)
        if ack:
            return ack.result == mavutil.mavlink.MAV_RESULT_ACCEPTED
    return False

# async def request_full_parameters():
#     global parameters, total_params
#     parameters.clear()
#     fetch_complete.clear()
#     total_params = 0
#     if connection and mav:
#         try:
#             mav.param_request_list_send(
#                 connection.target_system, connection.target_component
#             )
#             while True:
#                 msg = await asyncio.to_thread(connection.recv_match, type='PARAM_VALUE', blocking=True, timeout=5)
#                 if msg:
#                     parameters[msg.param_id] = {
#                         'value': msg.param_value,
#                     }
#                     total_params = msg.param_count
#                     if len(parameters) == total_params:
#                         break
#                 else:
#                     if total_params > 0 and len(parameters) == total_params:
#                         break
#                     if len(parameters) > 0:
#                         await asyncio.sleep(1)
#                     else:
#                         break
#         except Exception as e:
#             print(f"Error fetching parameters: {str(e)}")
#         finally:
#             fetch_complete.set()



async def request_full_parameters():
    global parameters, param_count, param_fetch_start, missing_parameters
    
    # Reset parameter fetch state
    parameters.clear()
    missing_parameters.clear()
    param_count = 0
    param_fetch_start = time.time()
    
    try:
        # Request parameter list
        if connection and connection.mav:
            connection.mav.param_request_list_send(
                connection.target_system,
                connection.target_component
            )
            logger.info("Parameter list requested")
            
            while True:
                if time.time() - param_fetch_start > FETCH_TIMEOUT:
                    logger.warning("Parameter fetch timed out")
                    break
                    
                msg = await asyncio.to_thread(
                    connection.recv_match,
                    type=['PARAM_VALUE', 'STATUSTEXT'],
                    blocking=True,
                    timeout=PARAM_TIMEOUT
                )
                
                if msg is None:
                    # Check if we have all parameters
                    if param_count > 0 and len(parameters) >= param_count:
                        break
                    continue

                if msg.get_type() == 'PARAM_VALUE':
                    # Update total parameter count
                    param_count = msg.param_count
                    
                    # Extract parameter information
                    param_id = msg.param_id.decode('utf-8') if isinstance(msg.param_id, bytes) else msg.param_id
                    param_type = msg.param_type
                    param_value = get_param_value(msg)
                    
                    # Store parameter
                    parameters[param_id] = {
                        'value': param_value,
                        'type': param_type,
                        'index': msg.param_index
                    }
                    
                    logger.debug(f"Received parameter {len(parameters)}/{param_count}: {param_id} = {param_value}")

            # After initial fetch, check for missing parameters
            if param_count > 0:
                expected_indices = set(range(param_count))
                received_indices = {param['index'] for param in parameters.values()}
                missing_indices = expected_indices - received_indices
                
                # Request missing parameters individually
                for idx in missing_indices:
                    missing_parameters.add(idx)
                
                await request_missing_parameters()

    except Exception as e:
        logger.error(f"Error fetching parameters: {e}")
    finally:
        fetch_complete.set()

def get_param_value(msg):
    """Convert parameter value based on its type."""
    try:
        if msg.param_type == mavutil.mavlink.MAV_PARAM_TYPE_REAL32:
            return float(msg.param_value)
        elif msg.param_type == mavutil.mavlink.MAV_PARAM_TYPE_REAL64:
            return float(msg.param_value)
        elif msg.param_type == mavutil.mavlink.MAV_PARAM_TYPE_INT8:
            return int(msg.param_value)
        elif msg.param_type == mavutil.mavlink.MAV_PARAM_TYPE_UINT8:
            return int(msg.param_value) & 0xFF
        elif msg.param_type == mavutil.mavlink.MAV_PARAM_TYPE_INT16:
            return int(msg.param_value)
        elif msg.param_type == mavutil.mavlink.MAV_PARAM_TYPE_UINT16:
            return int(msg.param_value) & 0xFFFF
        elif msg.param_type == mavutil.mavlink.MAV_PARAM_TYPE_INT32:
            return int(msg.param_value)
        elif msg.param_type == mavutil.mavlink.MAV_PARAM_TYPE_UINT32:
            return int(msg.param_value) & 0xFFFFFFFF
        else:
            return msg.param_value
    except Exception as e:
        logger.error(f"Error converting parameter value: {e}")
        return msg.param_value

async def request_missing_parameters():
    """Request any missing parameters individually."""
    global missing_parameters, retry_count
    
    while missing_parameters and any(retry_count[idx] < MAX_RETRIES for idx in missing_parameters):
        for idx in list(missing_parameters):
            if retry_count[idx] >= MAX_RETRIES:
                continue
                
            try:
                connection.mav.param_request_read_send(
                    connection.target_system,
                    connection.target_component,
                    b'',  # empty param_id indicates index-based request
                    idx
                )
                retry_count[idx] += 1
                
                msg = await asyncio.to_thread(
                    connection.recv_match,
                    type='PARAM_VALUE',
                    blocking=True,
                    timeout=PARAM_TIMEOUT
                )
                
                if msg and msg.param_index == idx:
                    param_id = msg.param_id.decode('utf-8') if isinstance(msg.param_id, bytes) else msg.param_id
                    param_value = get_param_value(msg)
                    
                    parameters[param_id] = {
                        'value': param_value,
                        'type': msg.param_type,
                        'index': msg.param_index
                    }
                    
                    missing_parameters.remove(idx)
                    logger.info(f"Retrieved missing parameter {param_id}")
                    
            except Exception as e:
                logger.error(f"Error requesting parameter index {idx}: {e}")
                
        await asyncio.sleep(0.1)  # Prevent overwhelming the connection
    
    if missing_parameters:
        logger.warning(f"Could not retrieve {len(missing_parameters)} parameters after {MAX_RETRIES} retries")


# @app.route("/set_parameter", methods=['POST'])
# async def set_parameter():
#     data = await request.get_json()
#     param_id = data.get('param_id')
#     param_value = data.get('param_value')
    
#     if not param_id or param_value is None:
#         return jsonify({"error": "Missing param_id or param_value"}), 400
    
#     if connection and mav:
#         try:
#             # Convert param_value to float (MAVLink uses float for all parameters)
#             param_value_float = float(param_value)
            
#             # Send parameter set command
#             mav.param_set_send(
#                 connection.target_system,
#                 connection.target_component,
#                 param_id.encode('utf-8'),
#                 param_value_float,
#                 mavutil.mavlink.MAV_PARAM_TYPE_REAL32
#             )
            
#             # Wait for PARAM_VALUE message to confirm the change
#             msg = await asyncio.to_thread(connection.recv_match, type='PARAM_VALUE', blocking=True, timeout=5)
#             if msg and msg.param_id == param_id:
#                 # Update local parameter dictionary
#                 parameters[param_id]['value'] = msg.param_value
#                 return jsonify({"status": "success", "new_value": msg.param_value}), 200
#             else:
#                 return jsonify({"error": "Parameter set failed or timed out"}), 500
#         except Exception as e:
#             return jsonify({"error": str(e)}), 500
#     else:
#         return jsonify({"error": "No active connection"}), 400
    
async def set_parameter(param_id, param_value):
    """Set parameter with proper type handling and verification"""
    try:
        if param_id not in parameters:
            return False, "Parameter not found"

        # Get original parameter info
        param_info = parameters[param_id]
        param_type = param_info['type'] if isinstance(param_info, dict) else mavutil.mavlink.MAV_PARAM_TYPE_REAL32

        # Send parameter set command
        connection.mav.param_set_send(
            connection.target_system,
            connection.target_component,
            param_id.encode('utf-8'),
            float(param_value),  # MAVLink expects float
            param_type
        )

        # Wait for confirmation
        start_time = time.time()
        while time.time() - start_time < 5:  # 5 second timeout
            msg = await asyncio.to_thread(connection.recv_match, 
                type=['PARAM_VALUE'],
                blocking=True,
                timeout=1
            )
            
            if msg and msg.get_type() == 'PARAM_VALUE':
                received_param_id = msg.param_id.decode('utf-8') if isinstance(msg.param_id, bytes) else msg.param_id
                if received_param_id == param_id:
                    # Update local parameter store
                    if isinstance(param_info, dict):
                        param_info['value'] = msg.param_value
                    else:
                        parameters[param_id] = {
                            'value': msg.param_value,
                            'type': msg.param_type
                        }
                    logger.info(f"Parameter {param_id} updated to {msg.param_value}")
                    return True, msg.param_value

        return False, "Timeout waiting for parameter confirmation"

    except Exception as e:
        logger.error(f"Error setting parameter {param_id}: {str(e)}")
        return False, str(e)

@app.route("/set_parameter", methods=['POST'])
async def set_parameter_route():
    """Endpoint to set a parameter value"""
    try:
        data = await request.get_json()
        param_id = data.get('param_id')
        param_value = data.get('param_value')

        if not param_id or param_value is None:
            return jsonify({"status": "error", "message": "Missing param_id or param_value"}), 400

        success, result = await set_parameter(param_id, param_value)
        
        if success:
            return jsonify({
                "status": "success",
                "param_id": param_id,
                "new_value": result
            })
        else:
            return jsonify({
                "status": "error",
                "message": result
            }), 500

    except Exception as e:
        logger.error(f"Error in set_parameter endpoint: {str(e)}")
        return jsonify({"status": "error", "message": str(e)}), 500


# async def update_parameters_periodically():
#     while True:
#         await request_full_parameters()
#         await asyncio.sleep(15)

async def update_parameter_value(msg):
    """Update parameter value when received from drone"""
    try:
        if msg.get_type() == 'PARAM_VALUE':
            param_id = msg.param_id.decode('utf-8') if isinstance(msg.param_id, bytes) else msg.param_id
            
            # Update parameter in our store
            if param_id in parameters:
                if isinstance(parameters[param_id], dict):
                    parameters[param_id]['value'] = msg.param_value
                else:
                    parameters[param_id] = {
                        'value': msg.param_value,
                        'type': msg.param_type
                    }
                logger.debug(f"Parameter {param_id} updated to {msg.param_value}")
                return True
    except Exception as e:
        logger.error(f"Error updating parameter value: {str(e)}")
    return False

@app.before_serving
async def startup():
    app.add_background_task(update_parameter_value)
    asyncio.create_task(send_heartbeat())
    
@app.route("/fetch_parameters")
async def fetch_parameters():
    if not fetch_complete.is_set():
        asyncio.create_task(request_full_parameters())
    return jsonify({"status": "Requesting parameters"})

# @app.route("/get_parameters")
# async def get_parameters():
#     if not fetch_complete.is_set():
#         return jsonify({"status": "fetching", "parameters": [], "total": total_params})
#     return jsonify({
#         "status": "complete", 
#         "parameters": list(parameters.items()), 
#         "total": total_params
#     })


@app.route("/get_parameters")
async def get_parameters():
    """Endpoint to get current parameter state."""
    if not fetch_complete.is_set():
        return jsonify({
            "status": "fetching",
            "parameters": list(parameters.items()),
            "total": param_count,
            "received": len(parameters),
            "missing": len(missing_parameters)
        })
    return jsonify({
        "status": "complete",
        "parameters": list(parameters.items()),
        "total": param_count,
        "received": len(parameters),
        "missing": len(missing_parameters)
    })



async def process_mavlink_message(msg):
    if msg.get_type() == 'STATUSTEXT':
        severity = msg.severity
        ap_message = {
            "text": f"AP: {msg.text}",
            "severity": severity
        }
        async with ap_messages_lock:
            ap_messages.append(ap_message)
        logger.info(f"AP Message (Severity {severity}): {msg.text}")
    elif msg.get_type() == 'COMMAND_ACK':
        cmd_name = mavutil.mavlink.enums['MAV_CMD'][msg.command].name
        result_name = mavutil.mavlink.enums['MAV_RESULT'][msg.result].name
        cmd_message = f"Got COMMAND_ACK: {cmd_name}: {result_name}"
        print(cmd_message)

async def request_data_streams():
    if connection and mav:
        # Request all data streams
        mav.request_data_stream_send(
            connection.target_system, connection.target_component,
            mavutil.mavlink.MAV_DATA_STREAM_ALL, 10, 1
        )

        # Request specific message intervals
        message_types = [
            mavutil.mavlink.MAVLINK_MSG_ID_HEARTBEAT,mavutil.mavlink.MAVLINK_MSG_ID_RADIO_STATUS,mavutil.mavlink.MAVLINK_MSG_ID_BATTERY_STATUS,mavutil.mavlink.MAVLINK_MSG_ID_GPS_RAW_INT,mavutil.mavlink.MAVLINK_MSG_ID_GLOBAL_POSITION_INT,
            mavutil.mavlink.MAVLINK_MSG_ID_ATTITUDE,mavutil.mavlink.MAVLINK_MSG_ID_VFR_HUD,mavutil.mavlink.MAVLINK_MSG_ID_RC_CHANNELS, mavutil.mavlink.MAVLINK_MSG_ID_DISTANCE_SENSOR,
            mavutil.mavlink.MAVLINK_MSG_ID_AUTOPILOT_VERSION,mavutil.mavlink.MAVLINK_MSG_ID_HOME_POSITION,mavutil.mavlink.MAVLINK_MSG_ID_SERVO_OUTPUT_RAW,mavutil.mavlink.MAVLINK_MSG_ID_EXTENDED_SYS_STATE
        ]

        for msg_id in message_types:
            mav.command_long_send(
                connection.target_system, connection.target_component,
                mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
                0, msg_id, 100000, 0, 0, 0, 0, 0 
            )

        # Request additional data streams with specific rates
        message_rates = {
            mavutil.mavlink.MAV_DATA_STREAM_EXTENDED_STATUS: 2,mavutil.mavlink.MAV_DATA_STREAM_POSITION: 2,mavutil.mavlink.MAV_DATA_STREAM_EXTRA1: 4,mavutil.mavlink.MAV_DATA_STREAM_EXTRA2: 4,mavutil.mavlink.MAV_DATA_STREAM_EXTRA3: 2,
            mavutil.mavlink.MAV_DATA_STREAM_RAW_SENSORS: 2,mavutil.mavlink.MAV_DATA_STREAM_RC_CHANNELS: 2,
        }
        
        for stream_id, rate in message_rates.items():
            mav.request_data_stream_send(
                connection.target_system, connection.target_component,
                stream_id, rate, 1
            )
            logger.info(f"Requested data stream {stream_id} at {rate} Hz")
            await asyncio.sleep(0.1)

@app.route('/messages', methods=['GET'])
async def get_ap_messages():
    async with ap_messages_lock:
        return jsonify(list(ap_messages))

@app.route('/ap_messages/clear', methods=['POST'])
async def clear_ap_messages():
    async with ap_messages_lock:
        ap_messages.clear()
    return jsonify({"status": "AP messages cleared"}), 200

async def message_handler():
    global accel_calibration_status
    while True:
        if connection:
            try:
                msg = await asyncio.to_thread(connection.recv_match, blocking=True, timeout=0.1)
                if msg:
                    msg_type = msg.get_type()

                    if msg_type == 'HEARTBEAT':
                        accel_calibration_status["connected"] = True

                    elif msg_type == 'STATUSTEXT':
                        text = msg.text

                        if isinstance(text, bytes):
                            text = text.decode('utf-8')
                        logger.info(f"STATUSTEXT: {text}")
                        if ('calib' in text.lower() or 'radio' in text.lower() or 'rc' in text.lower()):
                            radio_calibration_status["status_message"] = text
                            logger.info(f"STATUSTEXT: {text}")
                        accel_calibration_status["message"] = text

                    elif msg_type == 'COMMAND_ACK':
                        cmd_name = mavutil.mavlink.enums['MAV_CMD'][msg.command].name
                        result_name = MAV_RESULT_MAP.get(msg.result, str(msg.result))

                        command_queue.ack_command(msg.command)

                        update_calibration_status(msg.command, msg.result)
                        radio_calibration_status["last_command_ack"] = {
                            "command": msg.command,
                            "result": msg.result,
                            "time": time.time()
                        }

                        if msg.command == mavutil.mavlink.MAV_CMD_PREFLIGHT_CALIBRATION:
                            if radio_calibration_status["saving_parameters"]:
                                pass
                            elif msg.result == mavutil.mavlink.MAV_RESULT_ACCEPTED:
                                if radio_calibration_status["is_calibrating"]:
                                    radio_calibration_status["status_message"] = "Calibration in progress. Move sticks to extremes."
                                else:
                                    radio_calibration_status["status_message"] = "Calibration completed."
                            else:
                                if not radio_calibration_status["saving_parameters"]:
                                    radio_calibration_status["status_message"] = f"Calibration command failed: result {msg.result}"

                        logger.info(f"Got COMMAND_ACK: {cmd_name}: {result_name}")
                        command_queue.ack_command(msg.command)

                    elif msg_type == 'AUTOPILOT_VERSION':
                        flight_sw_version = ".".join(str(x) for x in msg.flight_sw_version[:3])
                        accel_calibration_status["version"] = f"ArduCopter V{flight_sw_version}"
                        logger.info(f"Drone version: {accel_calibration_status['version']}")

                    elif msg_type == 'RC_CHANNELS':
                        rc_values['channel_count'] = msg.chancount
                        rc_values['last_update'] = time.time()
                        
                        for i in range(msg.chancount):
                            chan_num = i + 1
                            value = getattr(msg, f'chan{chan_num}_raw', None)
                            if value is not None:
                                rc_values['channels'][chan_num] = value
                                
                                if radio_calibration_status["is_calibrating"] and not radio_calibration_status["saving_parameters"]:
                                    if chan_num not in radio_calibration_status["calibration_data"]:
                                        radio_calibration_status["calibration_data"][chan_num] = {
                                            "min": value,
                                            "max": value,
                                            "trim": value
                                        }
                                    else:
                                        cal_data = radio_calibration_status["calibration_data"][chan_num]
                                        cal_data["min"] = min(cal_data["min"], value)
                                        cal_data["max"] = max(cal_data["max"], value)
                                        if radio_calibration_status["saving_parameters"]:
                                            cal_data["trim"] = value

                    elif msg_type == 'PARAM_VALUE':
                        if radio_calibration_status["saving_parameters"]:
                            param_id = msg.param_id.decode('utf-8') if isinstance(msg.param_id, bytes) else msg.param_id
                            if param_id.startswith('RC') and ('MIN' in param_id or 'MAX' in param_id or 'TRIM' in param_id):
                                logger.info(f"Parameter {param_id} saved successfully")
            except Exception as e:
                logger.error(f"Message handling error: {e}")
        await asyncio.sleep(0.01)





async def upload_mission(waypoints):
    if not connection:
        return False

    # Clear any existing mission
    connection.mav.mission_clear_all_send(connection.target_system, connection.target_component)
    ack = await asyncio.to_thread(connection.recv_match, type='MISSION_ACK', blocking=True, timeout=5)
    if not ack or ack.type != mavutil.mavlink.MAV_MISSION_ACCEPTED:
        return False

    # Send mission count
    connection.mav.mission_count_send(connection.target_system, connection.target_component, len(waypoints))

    # Send each waypoint
    for i, wp in enumerate(waypoints):
        msg = await asyncio.to_thread(connection.recv_match, type='MISSION_REQUEST', blocking=True, timeout=5)
        if not msg or msg.seq != i:
            return False
        
        # Send waypoint data
        command, param1, param2, param3, param4 = get_command_and_params(wp)
        connection.mav.mission_item_int_send(
            connection.target_system,
            connection.target_component,
            i,
            mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
            command,
            0, 1,
            param1, param2, param3, param4,
            int(wp['lat'] * 1e7),
            int(wp['lng'] * 1e7),
            wp['altitude']
        )

    # Wait for final acknowledgment
    ack = await asyncio.to_thread(connection.recv_match, type='MISSION_ACK', blocking=True, timeout=5)
    return ack and ack.type == mavutil.mavlink.MAV_MISSION_ACCEPTED

@app.route('/mission', methods=['POST'])
async def start_mission():
    # Use await to get the JSON body from the request
    request_data = await request.get_json()  
    waypoints = request_data['waypoints']
    
    if not connection:
        return jsonify({'status': 'No active connection'}), 400

    try:
        if not await upload_mission(waypoints):
            return jsonify({'status': 'Failed to upload mission'}), 500

        return jsonify({'status': 'Mission uploaded successfully'}), 200
    except Exception as e:
        return jsonify({'status': f'Error: {str(e)}'}), 500
    
@app.route('/mission', methods=['GET'])
async def read_mission():
    if not connection:
        return jsonify({'status': 'No active connection'}), 400

    try:
        # Request mission count
        connection.mav.mission_request_list_send(connection.target_system, connection.target_component)
        msg = await asyncio.to_thread(connection.recv_match, type='MISSION_COUNT', blocking=True, timeout=5)
        if not msg:
            return jsonify({'status': 'Failed to receive mission count'}), 500

        mission_items = []
        for i in range(msg.count):
            # Request each waypoint
            connection.mav.mission_request_int_send(connection.target_system, connection.target_component, i)
            wp = await asyncio.to_thread(connection.recv_match, type='MISSION_ITEM_INT', blocking=True, timeout=5)
            if not wp:
                return jsonify({'status': f'Failed to receive waypoint {i}'}), 500

            mission_items.append({
                'lat': wp.x / 1e7,
                'lng': wp.y / 1e7,
                'altitude': wp.z,
                'command': wp.command
            })

        return jsonify({'waypoints': mission_items}), 200

    except Exception as e:
        return jsonify({'status': f'Error: {str(e)}'}), 500

@app.route('/mission', methods=['DELETE'])
async def remove_mission():
    if not connection:
        return jsonify({'status': 'No active connection'}), 400
    try:
        connection.mav.mission_clear_all_send(connection.target_system, connection.target_component)
        ack = await asyncio.to_thread(connection.recv_match, type='MISSION_ACK', blocking=True, timeout=5)
        if not ack or ack.type != mavutil.mavlink.MAV_MISSION_ACCEPTED:
            return jsonify({'status': 'Failed to clear mission'}), 500
        return jsonify({'status': 'Mission cleared successfully'}), 200
    except Exception as e:
        logger.error(f"Error clearing mission: {str(e)}")
        return jsonify({'status': f'Failed to clear mission: {str(e)}'}), 500

@app.route('/set_position', methods=['POST'])
async def set_position():
    data = await request.get_json()
    latitude = data.get('latitude')
    longitude = data.get('longitude')
    altitude = data.get('altitude')

    if latitude is None or longitude is None or altitude is None:
        return jsonify({'error': 'Missing latitude, longitude or altitude'}), 400
    try:
        target_lat = int(latitude * 1e7)
        target_lon = int(longitude * 1e7)
        target_alt = float(altitude)
        connection.mav.set_position_target_global_int_send(
            0,
            connection.target_system,
            connection.target_component,
            mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
            int(0b0000111111111000),
            target_lat,target_lon,target_alt,0, 0, 0,0, 0, 0,0, 0
        )
        return jsonify({'status': 'Position set successfully'})
    except Exception as e:
        return jsonify({'error': str(e)}), 500





#Compass Calibration
def update_calibration_data(msg):
    global calibration_data
    msg_type = msg.get_type()
    
    if msg_type == "MAG_CAL_PROGRESS":
        compass_id = msg.compass_id
        for compass in calibration_data["compasses"]:
            if compass["id"] == compass_id:
                compass["progress"] = msg.completion_pct
                logger.debug(f"MAG_CAL_PROGRESS: Compass {compass_id} progress: {msg.completion_pct}%")
                break
    elif msg_type == "MAG_CAL_REPORT":
        compass_id = msg.compass_id
        for compass in calibration_data["compasses"]:
            if compass["id"] == compass_id:
                compass["report"] = {
                    "compass_id": msg.compass_id,
                    "cal_status": msg.cal_status,
                    "autosaved": msg.autosaved,
                    "fitness": msg.fitness,
                    "ofs_x": msg.ofs_x,
                    "ofs_y": msg.ofs_y,
                    "ofs_z": msg.ofs_z,
                }
                compass["calibrated"] = msg.cal_status == mavutil.mavlink.MAG_CAL_SUCCESS
                logger.info(f"MAG_CAL_REPORT: Compass {compass_id} calibration status: {msg.cal_status}")
                break
    elif msg_type == "STATUSTEXT":
        calibration_data["status_text"] = msg.text
        logger.info(f"STATUSTEXT: {msg.text}")
    elif msg_type == "HEARTBEAT":
        calibration_data["heartbeat"] = "Connected"

async def update_data_continuously():
    global connection
    while True:
        if connection:
            try:
                msg = connection.recv_msg()
                if msg:
                    update_calibration_data(msg)
            except Exception as e:
                logger.error(f"Error receiving message: {e}")
        else:
            logger.warning("No active connection to drone")
        await asyncio.sleep(0.1) 

@app.route('/start_calibration', methods=['POST'])
async def start_calibration():
    try:
        data = await request.json
        compass_mask = data.get("compass_mask", 0)
        
        connection.mav.command_long_send(
            connection.target_system,
            connection.target_component,
            mavutil.mavlink.MAV_CMD_DO_START_MAG_CAL,
            0,
            0,  # mag_mask
            compass_mask,  # retry
            1,  # autosave
            0,  # delay
            0, 0, 0
        )
        calibration_data["calibration_started"] = True
        for compass in calibration_data["compasses"]:
            compass["progress"] = 0
            compass["calibrated"] = False
            compass["report"] = None
        logger.info("Magnetometer calibration started")
        return jsonify({"status": "Magnetometer calibration started"}), 200
    except Exception as e:
        logger.error(f"Error starting calibration: {str(e)}")
        return jsonify({"error": str(e)}), 500

@app.route('/cancel_calibration', methods=['POST'])
async def cancel_calibration():
    try:
        connection.mav.command_long_send(
            connection.target_system,
            connection.target_component,
            mavutil.mavlink.MAV_CMD_DO_CANCEL_MAG_CAL,
            0, 0, 0, 0, 0, 0, 0, 0
        )
        calibration_data["calibration_started"] = False
        logger.info("Magnetometer calibration cancelled")
        return jsonify({"status": "Magnetometer calibration cancelled"}), 200
    except Exception as e:
        logger.error(f"Error cancelling calibration: {str(e)}")
        return jsonify({"error": str(e)}), 500

@app.route('/reboot_drone', methods=['POST'])
async def reboot_drone():
    try:
        connection.mav.command_long_send(
            connection.target_system,
            connection.target_component,
            mavutil.mavlink.MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN,
            0, 1, 0, 0, 0, 0, 0, 0
        )
        logger.info("Drone reboot command sent")
        return jsonify({"status": "Drone is rebooting"}), 200
    except Exception as e:
        logger.error(f"Error rebooting drone: {str(e)}")
        return jsonify({"error": str(e)}), 500

# @app.route('/calibration_status', methods=['GET'])
# async def calibration_status():
#     return jsonify(calibration_data), jsonify(calibration_status), 200

@app.route('/calibration_status', methods=['GET'])
async def calibration_status():
    return jsonify(calibration_data)





#Accel calibration 
def update_calibration_status(command, result):
    global accel_calibration_status
    cmd_name = MAV_CMD_MAP.get(command, str(command))
    result_name = MAV_RESULT_MAP.get(result, str(result))
    
    accel_calibration_status["last_command_result"] = f"{cmd_name}: {result_name}"
    
    if command == mavutil.mavlink.MAV_CMD_PREFLIGHT_CALIBRATION:
        if result == mavutil.mavlink.MAV_RESULT_ACCEPTED:
            accel_calibration_status["completed"] = True
            accel_calibration_status["in_progress"] = False
            accel_calibration_status["message"] = f"{accel_calibration_status['type'].capitalize()} calibration completed successfully"
        elif result == mavutil.mavlink.MAV_RESULT_TEMPORARILY_REJECTED:
            accel_calibration_status["message"] = f"{accel_calibration_status['type'].capitalize()} calibration temporarily rejected"
        elif result == mavutil.mavlink.MAV_RESULT_DENIED:
            accel_calibration_status["completed"] = False
            accel_calibration_status["in_progress"] = False
            accel_calibration_status["message"] = f"{accel_calibration_status['type'].capitalize()} calibration denied"
        elif result == mavutil.mavlink.MAV_RESULT_UNSUPPORTED:
            accel_calibration_status["completed"] = False
            accel_calibration_status["in_progress"] = False
            accel_calibration_status["message"] = f"{accel_calibration_status['type'].capitalize()} calibration unsupported"
        elif result == mavutil.mavlink.MAV_RESULT_FAILED:
            accel_calibration_status["completed"] = False
            accel_calibration_status["in_progress"] = False
            accel_calibration_status["message"] = f"{accel_calibration_status['type'].capitalize()} calibration failed"
        elif result == mavutil.mavlink.MAV_RESULT_IN_PROGRESS:
            accel_calibration_status["message"] = f"{accel_calibration_status['type'].capitalize()} calibration in progress"

async def start_accel_calibration(cal_type):
    global accel_calibration_status
    accel_calibration_status["in_progress"] = True
    accel_calibration_status["type"] = cal_type
    accel_calibration_status["message"] = f"Starting {cal_type} calibration"
    accel_calibration_status["completed"] = False
    accel_calibration_status["last_command_result"] = None

    if cal_type == "full":
        connection.mav.command_long_send(
            connection.target_system, connection.target_component,
            mavutil.mavlink.MAV_CMD_PREFLIGHT_CALIBRATION,
            0, 0, 0, 0, 0, 1, 0, 0
        )
    elif cal_type == "simple":
        connection.mav.command_long_send(
            connection.target_system, connection.target_component,
            mavutil.mavlink.MAV_CMD_PREFLIGHT_CALIBRATION,
            0, 0, 0, 0, 0, 4, 0, 0
        )
    elif cal_type == "level":
        connection.mav.command_long_send(
            connection.target_system, connection.target_component,
            mavutil.mavlink.MAV_CMD_PREFLIGHT_CALIBRATION,
            0, 0, 0, 0, 0, 2, 0, 0
        )
    logger.info(f"Sent {cal_type} calibration command")

@app.route('/calibrate', methods=['POST'])
async def calibrate():
    if not accel_calibration_status["connected"]:
        return jsonify({"error": "Not connected to drone"}), 400
    await start_accel_calibration("full")
    return jsonify({"message": "Full calibration started"})

@app.route('/simple_calibrate', methods=['POST'])
async def simple_calibrate():
    if not accel_calibration_status["connected"]:
        return jsonify({"error": "Not connected to drone"}), 400
    await start_accel_calibration("simple")
    return jsonify({"message": "Simple calibration started"})

@app.route('/calibrate_level', methods=['POST'])
async def level_calibrate():
    if not accel_calibration_status["connected"]:
        return jsonify({"error": "Not connected to drone"}), 400
    await start_accel_calibration("level")
    return jsonify({"message": "Level calibration started"})

@app.route('/set_acceldrone_position/<position>', methods=['POST'])
async def set_acceldrone_position(position):
    if not accel_calibration_status["connected"]:
        return jsonify({"error": "Not connected to drone"}), 400
    if position in ACCEL_CAL_POSITIONS:
        connection.mav.command_long_send(
            connection.target_system, connection.target_component,
            mavutil.mavlink.MAV_CMD_ACCELCAL_VEHICLE_POS,
            0, ACCEL_CAL_POSITIONS[position], 0, 0, 0, 0, 0, 0
        )
        accel_calibration_status["message"] = f"Position {position} set for accelerometer calibration"
        return jsonify({"message": accel_calibration_status["message"]})
    else:
        return jsonify({"error": "Invalid position"}), 400

@app.route('/accel_calibration_status', methods=['GET'])
async def get_calibration_status():
    return jsonify(accel_calibration_status)




#radio calibration - infrontend change /status -> /radiostatus, /calibration/start,save,cancel,status -> /radiocalibration/start,save,cancel,status, get_status -> get_radio_status, get_parameters -> get_radio_parameters, get_calibration_status -> get_radio_calibration_status
async def start_radio_calibration():
    """Start RC calibration process"""
    global radio_calibration_status
    try:
        # Request RC calibration - param4=1 for RC calibration
        await command_queue.add_command(
            mavutil.mavlink.MAV_CMD_PREFLIGHT_CALIBRATION,
            0, 0, 0, 1, 0, 0, 0  # param4 = 1 for RC calibration
        )
        radio_calibration_status["is_calibrating"] = True
        radio_calibration_status["current_step"] = "waiting_for_movement"
        radio_calibration_status["calibration_data"] = {}
        radio_calibration_status["status_message"] = "Calibration started. Move all sticks to their extreme positions."
        logger.info("RC calibration started")
        return True
    except Exception as e:
        logger.error(f"Failed to start calibration: {e}")
        radio_calibration_status["status_message"] = f"Failed to start calibration: {e}"
        return False
    
async def save_radio_calibration():
    """Save RC calibration data"""
    global radio_calibration_status
    try:
        radio_calibration_status["saving_parameters"] = True
        parameters_saved = True
        
        for channel, data in radio_calibration_status["calibration_data"].items():
            # Set parameters using PARAM_SET message
            for param_type, value in [("MIN", data["min"]), ("MAX", data["max"]), ("TRIM", data["trim"])]:
                param_id = f"RC{channel}_{param_type}"
                logger.info(f"Setting parameter {param_id} to {value}")
                connection.mav.param_set_send(
                    connection.target_system,
                    connection.target_component,
                    param_id.encode(),
                    float(value),
                    mavutil.mavlink.MAV_PARAM_TYPE_REAL32
                )
                # Wait for parameter acknowledgment
                await asyncio.sleep(0.1)
        
        # Wait for all parameters to be saved
        await asyncio.sleep(1)
        
        # Stop calibration only after parameters are saved
        await command_queue.add_command(
            mavutil.mavlink.MAV_CMD_PREFLIGHT_CALIBRATION,
            0, 0, 0, 0, 0, 0, 0  # All zeros to stop calibration
        )
        
        radio_calibration_status["is_calibrating"] = False
        radio_calibration_status["saving_parameters"] = False
        radio_calibration_status["status_message"] = "Calibration saved successfully"
        return True
        
    except Exception as e:
        logger.error(f"Failed to save calibration: {e}")
        radio_calibration_status["status_message"] = f"Failed to save calibration: {e}"
        radio_calibration_status["saving_parameters"] = False
        return False

async def cancel_radio_calibration():
    """Cancel RC calibration process"""
    global radio_calibration_status
    try:
        await command_queue.add_command(
            mavutil.mavlink.MAV_CMD_PREFLIGHT_CALIBRATION,
            0, 0, 0, 0, 0, 0, 0  # All zeros to stop calibration
        )
        radio_calibration_status["is_calibrating"] = False
        radio_calibration_status["status_message"] = "Calibration cancelled"
        return True
    except Exception as e:
        logger.error(f"Failed to cancel calibration: {e}")
        radio_calibration_status["status_message"] = f"Failed to cancel calibration: {e}"
        return False
    
@app.route('/radiocalibration/start', methods=['POST'])
async def start_calibration_endpoint():
    success = await start_radio_calibration()
    return jsonify({"success": success, "state": radio_calibration_status})

@app.route('/radiocalibration/save', methods=['POST'])
async def save_calibration_endpoint():
    success = await save_radio_calibration()
    return jsonify({"success": success, "state": radio_calibration_status})

@app.route('/radiocalibration/cancel', methods=['POST'])
async def cancel_calibration_endpoint():
    success = await cancel_radio_calibration()
    return jsonify({"success": success, "state": radio_calibration_status})

@app.route('/radiocalibration/status', methods=['GET'])
async def get_radio_calibration_status():
    return jsonify(radio_calibration_status)





async def main():
    global command_queue
    command_queue = CommandQueue()
    asyncio.create_task(connect_drone())
    asyncio.create_task(send_heartbeat())
    asyncio.create_task(message_handler())
    asyncio.create_task(command_queue.process_queue())
    asyncio.create_task(command_queue.retry_commands())

    config = Config()
    config.bind = ["127.0.0.1:5001"]
    # config.cors_allow_origins = ["http://localhost:3001", "http://127.0.0.1:3001"] #from radio calibration
    await serve(app, config)





if __name__ == '__main__':
    app.run(host="0.0.0.0", port=5001)