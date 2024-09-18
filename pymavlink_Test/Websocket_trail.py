from flask import Flask, jsonify, request
from flask_cors import CORS
from pymavlink import mavutil
import serial.tools.list_ports
import threading
import time
import logging
import math
import platform
from queue import Queue


app = Flask(__name__)
CORS(app,supports_credentials=True)

connection = None
home_position_set = False
home_coordinates = {"lat": None, "lon": None, "alt": None}
drone_status = {
    "connection_status": "Disconnected",
    "system_id": None,
    "component_id": None,
    "firmware_type": None,
    "vehicle_type": None,
    "battery_status": {},
    "gps": None,
    "satellite_count": None,
    "hdop": None,
    "gps_fix_type": None,
    "armed": False,
    "firmware_version": None,
    "board_version": None,
    "rssi_dBm": None,
    "remrssi_dBm": None,
    "channel_outputs": {},
    "distance_to_home": None,
    "rangefinder": {},
    "rx_errors": None,
    "tx_buffer": None,
    "local_noise": None,
    "remote_noise": None,
    "attitude": {
        "pitch": None,
        "yaw": None,
        "roll": None
    },
    "groundspeed": None,
    "airspeed": None,
    "heading": None,
    "mav_mode": None,
    "mav_landed_state": None 
}

logging.basicConfig(level=logging.DEBUG)
logger = logging.getLogger(__name__)

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
            "voltage": msg.voltages[0] / 1000.0,
            "current": msg.current_battery / 100.0,
            "battery_remaining": msg.battery_remaining,
            "capacity_consumed": msg.current_consumed
        }

    elif msg_type == 'GPS_RAW_INT':
        drone_status['satellite_count'] = msg.satellites_visible
        drone_status['hdop'] = msg.eph / 100.0
        drone_status['gps_fix_type'] = msg.fix_type

    elif msg_type == 'GLOBAL_POSITION_INT':
        drone_status['gps'] = {
            "lat": msg.lat / 1e7,
            "lon": msg.lon / 1e7,
            "alt": msg.alt / 1000.0,
            "relative_alt": msg.relative_alt / 1000.0
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
            "distance": msg.current_distance / 100.0,
            "min_distance": msg.min_distance / 100.0,
            "max_distance": msg.max_distance / 100.0,
        }

    elif msg_type == 'AUTOPILOT_VERSION':
        drone_status['firmware_version'] = decode_firmware_version(msg.flight_sw_version)
        drone_status['board_version'] = msg.board_version

    elif msg_type == 'HOME_POSITION':
        drone_status['distance_to_home'] = calculate_distance_to_home()

    elif msg_type == 'SERVO_OUTPUT_RAW':
        drone_status['channel_outputs'] = {f"ch{i+1}_output": getattr(msg, f'servo{i+1}_raw') for i in range(8)}







def reset_telemetry_values():
    drone_status.update({
        "connection_status": "Disconnected",
    "system_id": None,
    "component_id": None,
    "firmware_type": None,
    "vehicle_type": None,
    "battery_status": {},
    "gps": None,
    "satellite_count": None,
    "hdop": None,
    "gps_fix_type": None,
    "armed": False,
    "firmware_version": None,
    "board_version": None,
    "rssi_dBm": None,
    "remrssi_dBm": None,
    "channel_outputs": {},
    "distance_to_home": None,
    "rangefinder": {},
    "rx_errors": None,
    "tx_buffer": None,
    "local_noise": None,
    "remote_noise": None,
    "attitude": {
        "pitch": None,
        "yaw": None,
        "roll": None
    },
    "groundspeed": None,
    "airspeed": None,
    "heading": None,
    "mav_mode": None,
    "mav_landed_state": None 
    })



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


def decode_mavlink_message(msg):
    if msg.get_type() == 'STATUSTEXT':
        return f"AP: {msg.text}"
    elif msg.get_type() == 'COMMAND_ACK':
        return f"Got COMMAND_ACK: {mavutil.mavlink.enums['MAV_CMD'][msg.command].name}: {mavutil.mavlink.enums['MAV_RESULT'][msg.result].name}"
    else:
        return f"Received {msg.get_type()} message"
    


def request_data_streams():
    if connection:
        connection.mav.request_data_stream_send(
            connection.target_system, connection.target_component,
            mavutil.mavlink.MAV_DATA_STREAM_ALL, 10, 1
        )
        message_types = [
            mavutil.mavlink.MAVLINK_MSG_ID_HEARTBEAT,
            mavutil.mavlink.MAVLINK_MSG_ID_RADIO_STATUS,
            mavutil.mavlink.MAVLINK_MSG_ID_BATTERY_STATUS,
            mavutil.mavlink.MAVLINK_MSG_ID_GPS_RAW_INT,
            mavutil.mavlink.MAVLINK_MSG_ID_GLOBAL_POSITION_INT,
            mavutil.mavlink.MAVLINK_MSG_ID_ATTITUDE,
            mavutil.mavlink.MAVLINK_MSG_ID_VFR_HUD,
            mavutil.mavlink.MAVLINK_MSG_ID_RC_CHANNELS,
            mavutil.mavlink.MAVLINK_MSG_ID_DISTANCE_SENSOR,
            mavutil.mavlink.MAVLINK_MSG_ID_AUTOPILOT_VERSION,
            mavutil.mavlink.MAVLINK_MSG_ID_HOME_POSITION,
            mavutil.mavlink.MAVLINK_MSG_ID_SERVO_OUTPUT_RAW,
            mavutil.mavlink.MAVLINK_MSG_ID_EXTENDED_SYS_STATE
        ]
        for msg_id in message_types:
            connection.mav.command_long_send(
                connection.target_system, connection.target_component,
                mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
                0, msg_id, 100000, 0, 0, 0, 0, 0  # 10 Hz for each message type
            )






@app.route('/connect', methods=['POST'])
def connect_drone():
    global connection, home_position_set
    data = request.json
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
        connection = mavutil.mavlink_connection(device, baud=baudrate if protocol == 'serial' else None)
        connection.wait_heartbeat(timeout=10)
        home_position_set = False
        request_data_streams()
        threading.Thread(target=fetch_drone_data, daemon=True).start()
        return jsonify({"status": f"Connected to drone via {protocol.upper()}"}), 200
    except Exception as e:
        return jsonify({"status": f"Failed to connect: {str(e)}"}), 500

@app.route('/disconnect', methods=['POST'])
def disconnect_drone():
    global connection
    if connection:
        connection.close()
        connection = None
        return jsonify({"status": "Disconnected from drone"}), 200
    else:
        return jsonify({"status": "No active connection"}), 400
    
def send_command_long(command, param1=0, param2=0, param3=0, param4=0, param5=0, param6=0, param7=0):
    if connection:
        connection.mav.command_long_send(
            connection.target_system, connection.target_component,
            command, 0, param1, param2, param3, param4, param5, param6, param7
        )
        # Wait for command acknowledgment
        ack = connection.recv_match(type='COMMAND_ACK', blocking=True, timeout=5)
        if ack:
            return ack.result == mavutil.mavlink.MAV_RESULT_ACCEPTED
    return False



message_queue = Queue()

# def decode_mavlink_message(msg):
#     if msg.get_type() == 'STATUSTEXT':
#         return f"AP: {msg.text}"
#     elif msg.get_type() == 'COMMAND_ACK':
#         return f"Got COMMAND_ACK: {mavutil.mavlink.enums['MAV_CMD'][msg.command].name}: {mavutil.mavlink.enums['MAV_RESULT'][msg.result].name}"
#     else:
#         return f"Received {msg.get_type()} message"
    


def fetch_drone_data():
    global connection, drone_status
    while True:
        if connection:
            try:
                msg = connection.recv_match(blocking=True, timeout=1)
                if msg:
                    if msg.get_type() != 'BAD_DATA':
                        logger.debug(f"Received message: {msg.get_type()}")
                        update_drone_status(msg)
                        message_queue.put(decode_mavlink_message(msg))
                else:
                    logger.warning("No message received")
            except Exception as e:
                logger.error(f"Error receiving data: {e}")
                drone_status["connection_status"] = "Error"
        else:
            logger.warning("No active connection")
            reset_telemetry_values()
            time.sleep(1)

def set_mode(mode):
    if connection:
        if mode not in connection.mode_mapping():
            print(f'Unknown mode : {mode}')
            print(f'Try:', list(connection.mode_mapping().keys()))
            return False
        mode_id = connection.mode_mapping()[mode]
        return send_command_long(
            mavutil.mavlink.MAV_CMD_DO_SET_MODE, 
            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            mode_id
        )
    return False

@app.route('/command', methods=['POST'])
def command():
    data = request.json
    command = data['command']
    
    if command == 'arm':
        success = send_command_long(mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 1)
    elif command == 'disarm':
        success = send_command_long(mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0)
    elif command == 'takeoff':
        altitude = float(data['altitude'])
        success = send_command_long(mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, altitude)
    elif command == 'land':
        success = send_command_long(mavutil.mavlink.MAV_CMD_NAV_LAND)
    elif command == 'rtl':
        success = set_mode('RTL')
    elif command == 'set_home':
        latitude = float(data['latitude'])
        longitude = float(data['longitude'])
        altitude = float(data['altitude'])
        success = send_command_long(mavutil.mavlink.MAV_CMD_DO_SET_HOME, 0, 0, 0, 0, latitude, longitude, altitude)
    elif command == 'set_mode':
        mode = data['mode']
        success = set_mode(mode)
    else:
        return jsonify({'status': 'unknown_command'}), 400
    
    return jsonify({'status': 'success' if success else 'failed'}), 200 if success else 500

@app.route('/mission', methods=['POST'])
def start_mission():
    waypoints = request.json['waypoints']
    
    if not connection:
        return jsonify({'status': 'No active connection'}), 400

    try:
        # Clear any existing mission
        connection.mav.mission_clear_all_send(connection.target_system, connection.target_component)
        
        # Wait for acknowledgment
        ack = connection.recv_match(type='MISSION_ACK', blocking=True, timeout=5)
        if not ack or ack.type != mavutil.mavlink.MAV_MISSION_ACCEPTED:
            return jsonify({'status': 'Failed to clear existing mission'}), 500

        # Upload new mission
        for i, wp in enumerate(waypoints):
            connection.mav.mission_item_send(
                connection.target_system,
                connection.target_component,
                i,  # seq
                mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
                mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
                0,  # current
                1,  # autocontinue
                0, 0, 0, 0,  # param1-4
                wp['lat'], wp['lng'], wp['altitude']
            )

        # Start mission
        connection.mav.command_long_send(
            connection.target_system,
            connection.target_component,
            mavutil.mavlink.MAV_CMD_MISSION_START,
            0, 0, 0, 0, 0, 0, 0, 0
        )

        return jsonify({'status': 'Mission started successfully'}), 200
    except Exception as e:
        return jsonify({'status': f'Failed to start mission: {str(e)}'}), 500
    


@app.route('/messages', methods=['GET'])
def get_messages():
    messages = []
    while not message_queue.empty():
        messages.append(message_queue.get())
    return jsonify(messages)

@app.route('/com_ports', methods=['GET'])
def get_com_ports():
    ports = list_serial_ports()
    return jsonify({"ports": ports}), 200

@app.route('/status', methods=['GET'])
def get_status():
    return jsonify(drone_status), 200

if __name__ == '__main__':
    app.run(host="0.0.0.0", port=5001, debug=True)