from flask import Flask, jsonify, request
from flask_cors import CORS
from pymavlink import mavutil
import math

app = Flask(__name__)
CORS(app)

# Establish MAVLink connection
connection = mavutil.mavlink_connection('udp:127.0.0.1:14550')
connection.wait_heartbeat()
print("Heartbeat received")

home_position = None  # Store home position for distance calculation
parameters = {}  # Dictionary to store parameter data

# Autopilot firmware types
AUTOPILOT_NAMES = {
    0: "Generic",
    1: "Reserved",
    2: "Pixhawk",
    3: "Slugs",
    4: "ArduPilot",
    5: "OpenPilot",
    6: "Generic Wookong",
    7: "Generic APM",
    8: "Generic PX4",
    9: "BBBlue",
    10: "Navio2",
    11: "AutoQuad",
    12: "Pixhawk",
    13: "Generic FlightGear",
    14: "Generic AirSim",
    15: "Generic SmartAP",
    16: "Generic AirRails"
}

def calculate_distance(lat1, lon1, lat2, lon2):
    """Calculate distance between two GPS coordinates in meters"""
    R = 6371000  # Earth radius in meters
    phi1, phi2 = math.radians(lat1), math.radians(lat2)
    delta_phi = math.radians(lat2 - lat1)
    delta_lambda = math.radians(lon2 - lon1)
    a = math.sin(delta_phi / 2)**2 + math.cos(phi1) * math.cos(phi2) * math.sin(delta_lambda / 2)**2
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))
    return R * c

def send_command_long(command, param1=0, param2=0, param3=0, param4=0, param5=0, param6=0, param7=0):
    """Helper function to send a MAVLink command_long message"""
    connection.mav.command_long_send(
        connection.target_system,
        connection.target_component,
        command,
        0,  # Confirmation
        param1,
        param2,
        param3,
        param4,
        param5,
        param6,
        param7
    )
    print(f"Sent command: {command}, Params: {param1}, {param2}, {param3}, {param4}, {param5}, {param6}, {param7}")

def get_autopilot_name(firmware_type):
    """Get the autopilot name from the firmware type"""
    return AUTOPILOT_NAMES.get(firmware_type, "Unknown Autopilot")

@app.route('/arm', methods=['POST'])
def arm():
    """Arm the drone"""
    send_command_long(mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 1)
    return jsonify({"status": "armed"})

@app.route('/disarm', methods=['POST'])
def disarm():
    """Disarm the drone"""
    send_command_long(mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0)
    return jsonify({"status": "disarmed"})

@app.route('/takeoff', methods=['POST'])
def takeoff():
    """Takeoff without altitude input"""
    send_command_long(mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, 50)  # Default altitude
    return jsonify({"status": "taking off"})

@app.route('/land', methods=['POST'])
def land():
    """Land the drone"""
    send_command_long(mavutil.mavlink.MAV_CMD_NAV_LAND)
    return jsonify({"status": "landing"})

@app.route('/gps', methods=['GET'])
def get_gps():
    """Get GPS data (latitude, longitude, altitude)"""
    msg = connection.recv_match(type='GLOBAL_POSITION_INT', blocking=True)
    lat = msg.lat / 1e7
    lon = msg.lon / 1e7
    alt = msg.alt / 1e3  # Altitude in meters
    return jsonify({
        "latitude": lat,
        "longitude": lon,
        "altitude": alt
    })

@app.route('/heartbeat', methods=['GET'])
def get_heartbeat():
    """Get firmware type from the heartbeat message"""
    heartbeat = connection.recv_match(type='HEARTBEAT', blocking=True)
    firmware_type = heartbeat.autopilot  # Autopilot firmware type
    autopilot_name = get_autopilot_name(firmware_type)
    return jsonify({
        "heartbeat_received": True,
        "autopilot_name": autopilot_name
    })

@app.route('/telemetry', methods=['GET'])
def get_telemetry():
    """Get telemetry data: airspeed, groundspeed, altitude, battery, etc."""
    global home_position
    
    # Get airspeed, groundspeed, altitude, etc.
    vfr_hud = connection.recv_match(type='VFR_HUD', blocking=True)
    global_position = connection.recv_match(type='GLOBAL_POSITION_INT', blocking=True)
    sys_status = connection.recv_match(type='SYS_STATUS', blocking=True)
    gps_raw = connection.recv_match(type='GPS_RAW_INT', blocking=True)

    # Store home position if not already set
    if home_position is None and connection.recv_match(type='HOME_POSITION', blocking=False):
        home_position = connection.recv_match(type='HOME_POSITION', blocking=True)

    telemetry_data = {
        "airspeed": vfr_hud.airspeed,  # in m/s
        "groundspeed": vfr_hud.groundspeed,  # in m/s
        "relative_altitude": global_position.relative_alt / 1000.0,  # in meters
        "battery_voltage": sys_status.voltage_battery / 1000.0,  # in volts
        "battery_remaining": sys_status.battery_remaining,  # in percentage
        "satellites_visible": gps_raw.satellites_visible,  # satellite count

    }

    return jsonify(telemetry_data)


@app.route('/parameters', methods=['GET'])
def get_parameters():
    """Get all parameters from the drone"""
    parameters = {}
    connection.mav.param_request_list_send(
        connection.target_system, connection.target_component
    )
    while True:
        message = connection.recv_match(type='PARAM_VALUE', blocking=True)
        if message is None:
            break
        param_id = message.param_id.strip()
        param_value = message.param_value
        parameters[param_id] = param_value

        # Stop once we've received the last parameter
        if message.param_index + 1 >= message.param_count:
            break

    return jsonify({"parameters": parameters})

if __name__ == '__main__':
    app.run(host='0.0.0.0', port=5000)