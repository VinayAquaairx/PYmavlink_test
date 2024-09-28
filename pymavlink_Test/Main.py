from quart import Quart, jsonify, request
from quart_cors import cors
from pymavlink import mavutil
import serial.tools.list_ports
import asyncio
import time
from pymavlink.dialects.v20 import ardupilotmega as mavlink2
import logging
import math
import platform
from collections import deque
import socket

app = Quart(__name__)
app = cors(app, 
    allow_origin=["http://localhost:5173", "http://localhost:3000"],
    allow_credentials=True,
    allow_methods=["GET", "POST", "PUT", "DELETE", "OPTIONS"]
)

connection = None
mav = None
home_position_set = False
home_coordinates = {"lat": None, "lon": None, "alt": None}
parameters = {}
last_heartbeat_time = 0
packet_count = 0
total_packets = 0
connection_quality = 0


drone_status = {
    "connection_status": "Disconnected","system_id": None,"component_id": None,"firmware_type": None,"vehicle_type": None,"battery_status": {},"gps": None,"satellite_count": None,"hdop": None,"gps_fix_type": None,
    "armed": False,"firmware_version": None,"board_version": None,"rssi_dBm": None,"remrssi_dBm": None,"channel_outputs": {},"distance_to_home": None,"rangefinder": {},"rx_errors": None,"tx_buffer": None,
    "local_noise": None,"remote_noise": None,"attitude": {"pitch": None,"yaw": None,"roll": None},"groundspeed": None,"airspeed": None,"heading": None,"mav_mode": None,"mav_landed_state": None 
}

logging.basicConfig(level=logging.DEBUG)
logger = logging.getLogger(__name__)

global_device = None
global_baudrate = None
global_protocol = None

MAX_AP_MESSAGES = 1000 
ap_messages = deque(maxlen=MAX_AP_MESSAGES)
ap_messages_lock = asyncio.Lock()

def list_serial_ports():
    if platform.system() == "Windows":
        ports = serial.tools.list_ports.comports()
        return [port.device for port in ports]
    else:  # Linux (Ubuntu)
        import glob
        return glob.glob('/dev/tty[A-Za-z]*')

def decode_firmware_version(version):
    major = (version >> 24) & 0xFF
    minor = (version >> 16) & 0xFF
    patch = (version >> 8) & 0xFF
    return f"{major}.{minor}.{patch}"


def update_drone_status(msg):
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
        drone_status['distance_to_home'] = calculate_distance_to_home()

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


def calculate_distance_to_home():
    if drone_status['gps']:
        R = 6371000
        lat1, lon1, lat2, lon2 = map(math.radians, [home_coordinates['lat'], home_coordinates['lon'], drone_status['gps']['lat'], drone_status['gps']['lon']])
        dlat = lat2 - lat1
        dlon = lon2 - lon1
        a = math.sin(dlat/2)**2 + math.cos(lat1) * math.cos(lat2) * math.sin(dlon/2)**2
        c = 2 * math.atan2(math.sqrt(a), math.sqrt(1-a))
        return R * c
    return None



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


        
# Modify existing connection logic (make sure the connection is set properly)
async def connect_to_drone(connection_string, baudrate=115200):
    global connection, mav
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
    
async def request_data_streams():
    if connection and mav:
        mav.request_data_stream_send(
            connection.target_system, connection.target_component,
            mavutil.mavlink.MAV_DATA_STREAM_ALL, 10, 1
        )

        message_types = [
            mavutil.mavlink.MAVLINK_MSG_ID_HEARTBEAT,mavutil.mavlink.MAVLINK_MSG_ID_RADIO_STATUS, mavutil.mavlink.MAVLINK_MSG_ID_BATTERY_STATUS,mavutil.mavlink.MAVLINK_MSG_ID_GPS_RAW_INT,
            mavutil.mavlink.MAVLINK_MSG_ID_GLOBAL_POSITION_INT,mavutil.mavlink.MAVLINK_MSG_ID_ATTITUDE,mavutil.mavlink.MAVLINK_MSG_ID_VFR_HUD,mavutil.mavlink.MAVLINK_MSG_ID_RC_CHANNELS,mavutil.mavlink.MAVLINK_MSG_ID_DISTANCE_SENSOR,
            mavutil.mavlink.MAVLINK_MSG_ID_AUTOPILOT_VERSION,mavutil.mavlink.MAVLINK_MSG_ID_HOME_POSITION,mavutil.mavlink.MAVLINK_MSG_ID_SERVO_OUTPUT_RAW,mavutil.mavlink.MAVLINK_MSG_ID_EXTENDED_SYS_STATE
        ]

        for msg_id in message_types:
            mav.command_long_send(
                connection.target_system, connection.target_component,
                mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
                0, msg_id, 100000, 0, 0, 0, 0, 0 
            )


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
    

async def upload_mission(waypoints):
    if not connection:
        return False
    # Clear any existing mission
    connection.mav.mission_clear_all_send(connection.target_system, connection.target_component)
    ack = await asyncio.to_thread(connection.recv_match, type='MISSION_ACK', blocking=True, timeout=5)
    if not ack or ack.type != mavutil.mavlink.MAV_MISSION_ACCEPTED:
        logger.error("Failed to clear existing mission")
        return False
    # Upload new mission
    connection.mav.mission_count_send(connection.target_system, connection.target_component, len(waypoints))
    for i, wp in enumerate(waypoints):
        msg = await asyncio.to_thread(connection.recv_match, type=['MISSION_REQUEST'], blocking=True, timeout=5)
        if not msg:
            logger.error(f"Failed to receive MISSION_REQUEST for waypoint {i}")
            return False 
        # Set command and parameters based on waypoint type
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
    # Wait for mission acceptance
    ack = await asyncio.to_thread(connection.recv_match, type='MISSION_ACK', blocking=True, timeout=5)
    if not ack or ack.type != mavutil.mavlink.MAV_MISSION_ACCEPTED:
        logger.error("Mission upload failed")
        return False

    logger.info("Mission uploaded successfully")
    return True

@app.route('/mission', methods=['POST'])
async def start_mission():
    waypoints = (await request.get_json())['waypoints']
    
    if not connection:
        return jsonify({'status': 'No active connection'}), 400
    try:
        if not await upload_mission(waypoints):
            return jsonify({'status': 'Failed to upload mission'}), 500
        connection.mav.mission_request_list_send(connection.target_system, connection.target_component)
        if not await set_mode('AUTO'):
            return jsonify({'status': 'Failed to set AUTO mode'}), 500
        if not await send_command_long(mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 1):
            return jsonify({'status': 'Failed to arm the drone'}), 500

        # Start mission
        if not await send_command_long(mavutil.mavlink.MAV_CMD_MISSION_START):
            return jsonify({'status': 'Failed to start mission'}), 500

        return jsonify({'status': 'Mission started successfully'}), 200
    except Exception as e:
        logger.error(f"Error starting mission: {str(e)}")
        return jsonify({'status': f'Failed to start mission: {str(e)}'}), 500



async def periodic_connection_check():
    while True:
        if connection:
            if time.time() - last_heartbeat_time > 5:  # 5 seconds without heartbeat
                logger.warning("No heartbeat received for 5 seconds. Attempting reconnection.")
                await reconnect_drone()
        await asyncio.sleep(1)


        

@app.route('/connect', methods=['POST'])
async def connect_drone():
    global connection, home_position_set, mav, global_device, global_baudrate, global_protocol, last_heartbeat_time
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
    try:
        # NEW: Store connection details globally
        global_device = device
        global_baudrate = baudrate if protocol == 'serial' else None
        global_protocol = protocol
        connection = mavutil.mavlink_connection(device, baud=global_baudrate, source_system=255, source_component=0, autoreconnect=True, timeout=60)
        mav = mavlink2.MAVLink(connection)
        mav.srcSystem = 255
        mav.srcComponent = 0
        await asyncio.to_thread(connection.wait_heartbeat, timeout=10)
        last_heartbeat_time = time.time()
        home_position_set = False
        request_data_streams()
        asyncio.create_task(fetch_drone_data())
        asyncio.create_task(send_heartbeat())
        return jsonify({"status": f"Connected to drone via {protocol.upper()}"}), 200
    except Exception as e:
        return jsonify({"status": f"Failed to connect: {str(e)}"}), 500

@app.route('/disconnect', methods=['POST'])
async def disconnect_drone():
    global connection
    if connection:
        connection.close()
        connection = None
        return jsonify({"status": "Disconnected from drone"}), 200
    else:
        return jsonify({"status": "No active connection"}), 400
    

async def send_heartbeat():
    while True:
        if connection and mav:
            try:
                mav.heartbeat_send(
                    mavutil.mavlink.MAV_TYPE_GCS,
                    mavutil.mavlink.MAV_AUTOPILOT_INVALID,
                    0, 0, 0
                )
            except Exception as e:
                logger.error(f"Error sending heartbeat: {e}")
        await asyncio.sleep(1)

async def reconnect_drone():
    global connection, mav, last_heartbeat_time
    max_retries = 5
    retry_delay = 5  # seconds

    for attempt in range(max_retries):
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
            return
        except Exception as e:
            logger.error(f"Failed to reconnect (attempt {attempt + 1}/{max_retries}): {str(e)}")
            if attempt < max_retries - 1:
                await asyncio.sleep(retry_delay)
    
    logger.error("Failed to reconnect after maximum attempts")


@app.get("/connection_status")
async def get_connection_status():
    if connection:
        return {"status": "Connected", "last_heartbeat": time.time() - last_heartbeat_time}
    else:
        return {"status": "Disconnected"}
    

async def fetch_drone_data():
    global connection, drone_status, last_heartbeat_time, packet_count, total_packets, connection_quality
    heartbeat_timeout = 3
    while True:
        if connection:
            try:
                msg = await asyncio.to_thread(connection.recv_match, blocking=True, timeout=1)
                if msg:
                    total_packets += 1
                    if msg.get_type() != 'BAD_DATA':
                        packet_count += 1
                        update_drone_status(msg)
                        await process_mavlink_message(msg)
                        if msg.get_type() == 'HEARTBEAT':
                            last_heartbeat_time = time.time()
                    connection_quality = (packet_count / total_packets) * 100 if total_packets > 0 else 0
                if time.time() - last_heartbeat_time > heartbeat_timeout:
                    logger.warning("Heartbeat lost. Clearing drone status.")
                    reset_telemetry_values()
                    await reconnect_drone()
            except Exception as e:
                logger.error(f"Error receiving data: {e}")
                drone_status["connection_status"] = "Error"
                reset_telemetry_values()
        else:
            logger.warning("No active connection")
            reset_telemetry_values()
            await asyncio.sleep(1)

async def get_home_position():
    connection.mav.command_long_send(
        connection.target_system,
        connection.target_component,
        mavutil.mavlink.MAV_CMD_GET_HOME_POSITION,
        0,  
        0, 0, 0, 0, 0, 0, 0 
    )
    msg = await asyncio.to_thread(connection.recv_match, type='HOME_POSITION', blocking=True)
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

# Request full parameter list from the drone
def request_full_parameters():
    if connection and mav:
        mav.param_request_list_send(
            connection.target_system, connection.target_component
        )
        # Wait for all parameters to be received
        while True:
            msg = connection.recv_match(type='PARAM_VALUE', blocking=True, timeout=5)
            if msg:
                parameters[msg.param_id] = msg.param_value  # Store parameter values
                parameters.put({
                    'param_id': msg.param_id,'param_value': msg.param_value,'param_index': msg.param_index,'param_count': msg.param_count
                })
            else:
                break

# Endpoint to retrieve parameters in the frontend
@app.get("/get_parameters")
async def get_parameters():
    return list(parameters.items())

@app.get("/fetch_parameters")
async def fetch_parameters():
    await request_full_parameters()
    return {"status": "Requesting parameters"}

@app.route('/status', methods=['GET'])
async def get_status():
    global drone_status, connection_quality
    drone_status["connection_quality"] = connection_quality
    return jsonify(drone_status), 200

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
        altitude = float(data['altitude'])
        success = await send_command_long(mavutil.mavlink.MAV_CMD_DO_SET_HOME, 0, 0, 0, 0, latitude, longitude, altitude/1000)
    elif command == 'set_mode':
        mode = data['mode']
        success = await set_mode(mode)
    else:
        return jsonify({'status': 'unknown_command'}), 400
    
    return jsonify({'status': 'success' if success else 'failed'}), 200 if success else 500

@app.route('/messages', methods=['GET'])
async def get_ap_messages():
    async with ap_messages_lock:
        return jsonify(list(ap_messages))

@app.route('/ap_messages/clear', methods=['POST'])
async def clear_ap_messages():
    async with ap_messages_lock:
        ap_messages.clear()
    return jsonify({"status": "AP messages cleared"}), 200

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
            target_lat,
            target_lon,
            target_alt,
            0, 0, 0,
            0, 0, 0,
            0, 0
        )
        return jsonify({'status': 'Position set successfully'})
    except Exception as e:
        return jsonify({'error': str(e)}), 500
    

@app.route('/com_ports', methods=['GET'])
async def get_com_ports():
    ports = list_serial_ports()
    return jsonify({"ports": ports}), 200

if __name__ == '__main__':
    app.run(host="0.0.0.0", port=5001)
