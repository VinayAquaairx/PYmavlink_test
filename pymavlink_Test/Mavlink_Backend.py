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
from collections import deque

app = Quart(__name__)
app = cors(app, 
    allow_origin=["http://localhost:5173", "http://localhost:3000"],
    allow_credentials=True,
    allow_methods=["GET", "POST", "PUT", "DELETE", "OPTIONS"]
)

last_heartbeat_time = 0
packet_count = 0
total_packets = 0
connection_quality = 0
total_params = 0
parameters = {}
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

async def fetch_parameter_file():
    logger.info("Fetching parameter file (placeholder)")


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
    global connection  # Make sure to use the global connection object
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

@app.route('/calibration_status', methods=['GET'])
async def calibration_status():
    return jsonify(calibration_data), 200


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
        await request_data_streams()
        await request_autopilot_capabilities()
        await send_banner_request()
        await fetch_parameter_file()
        await detect_compasses()
        asyncio.create_task(fetch_drone_data())
        asyncio.create_task(send_heartbeat())
        return jsonify({"status": f"Connected to drone via {protocol.upper()}"}), 200
    except Exception as e:
        return jsonify({"status": f"Failed to connect: {str(e)}"}), 500

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

async def request_full_parameters():
    global parameters, total_params
    parameters.clear()
    fetch_complete.clear()
    total_params = 0
    if connection and mav:
        try:
            mav.param_request_list_send(
                connection.target_system, connection.target_component
            )
            while True:
                msg = await asyncio.to_thread(connection.recv_match, type='PARAM_VALUE', blocking=True, timeout=5)
                if msg:
                    parameters[msg.param_id] = {
                        'value': msg.param_value,
                    }
                    total_params = msg.param_count
                    if len(parameters) == total_params:
                        break
                else:
                    if total_params > 0 and len(parameters) == total_params:
                        break
                    if len(parameters) > 0:
                        await asyncio.sleep(1)
                    else:
                        break
        except Exception as e:
            print(f"Error fetching parameters: {str(e)}")
        finally:
            fetch_complete.set()

@app.route("/set_parameter", methods=['POST'])
async def set_parameter():
    data = await request.get_json()
    param_id = data.get('param_id')
    param_value = data.get('param_value')
    
    if not param_id or param_value is None:
        return jsonify({"error": "Missing param_id or param_value"}), 400
    
    if connection and mav:
        try:
            # Convert param_value to float (MAVLink uses float for all parameters)
            param_value_float = float(param_value)
            
            # Send parameter set command
            mav.param_set_send(
                connection.target_system,
                connection.target_component,
                param_id.encode('utf-8'),
                param_value_float,
                mavutil.mavlink.MAV_PARAM_TYPE_REAL32
            )
            
            # Wait for PARAM_VALUE message to confirm the change
            msg = await asyncio.to_thread(connection.recv_match, type='PARAM_VALUE', blocking=True, timeout=5)
            if msg and msg.param_id == param_id:
                # Update local parameter dictionary
                parameters[param_id]['value'] = msg.param_value
                return jsonify({"status": "success", "new_value": msg.param_value}), 200
            else:
                return jsonify({"error": "Parameter set failed or timed out"}), 500
        except Exception as e:
            return jsonify({"error": str(e)}), 500
    else:
        return jsonify({"error": "No active connection"}), 400
    
async def update_parameters_periodically():
    while True:
        await request_full_parameters()
        await asyncio.sleep(15)

@app.before_serving
async def startup():
    app.add_background_task(update_parameters_periodically)
    asyncio.create_task(send_heartbeat())
    
@app.route("/fetch_parameters")
async def fetch_parameters():
    if not fetch_complete.is_set():
        asyncio.create_task(request_full_parameters())
    return jsonify({"status": "Requesting parameters"})

@app.route("/get_parameters")
async def get_parameters():
    if not fetch_complete.is_set():
        return jsonify({"status": "fetching", "parameters": [], "total": total_params})
    return jsonify({
        "status": "complete", 
        "parameters": list(parameters.items()), 
        "total": total_params
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





if __name__ == '__main__':
    app.run(host="0.0.0.0", port=5001)