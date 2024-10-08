from quart import Quart, jsonify, request
from quart_cors import cors
from pymavlink import mavutil
import asyncio

app = Quart(__name__)
app = cors(app, allow_origin="*")

# Create connections to both UDP and TCP for the drone
udp_connection = mavutil.mavlink_connection("udp:127.0.0.1:14550")
tcp_connection = mavutil.mavlink_connection("tcp:192.168.4.1:8888")

async def wait_for_heartbeat(connection):
    """Waits for a heartbeat from the drone."""
    while True:
        msg = connection.recv_match(type='HEARTBEAT', blocking=True, timeout=5)
        if msg:
            return True
        await asyncio.sleep(1)
    return False

async def start_calibration(connection):
    """Initiates accelerometer calibration."""
    try:
        await send_command(connection, mavutil.mavlink.MAV_CMD_PREFLIGHT_CALIBRATION, 1)
        return "Accelerometer calibration started"
    except Exception as e:
        return f"Error: {e}"

async def send_command(connection, command, param1=0, param2=0, param3=0, param4=0, param5=0, param6=0, param7=0):
    """Helper to send MAVLink commands."""
    connection.mav.command_long_send(
        connection.target_system,
        connection.target_component,
        command,
        0,
        param1, param2, param3, param4, param5, param6, param7
    )

@app.route('/connect', methods=['GET'])
async def connect():
    """Attempts to connect and waits for a heartbeat."""
    connected = await wait_for_heartbeat(udp_connection)
    if connected:
        return jsonify({"status": "Connected to drone via UDP"})
    return jsonify({"error": "Could not connect to drone"}), 500

@app.route('/calibrate/start', methods=['POST'])
async def calibrate_start():
    """Starts the accelerometer calibration."""
    response = await start_calibration(udp_connection)
    return jsonify({"status": response})

@app.route('/calibrate/position', methods=['POST'])
async def calibrate_position():
    """Sets the required position during calibration."""
    position = request.json.get("position")
    positions = {
        "level": 1, "left": 2, "right": 3,
        "nose_up": 4, "nose_down": 5, "tail_down": 6
    }
    pos_value = positions.get(position)
    if pos_value is None:
        return jsonify({"error": "Invalid position"}), 400
    await send_command(udp_connection, mavutil.mavlink.MAV_CMD_ACCELCAL_VEHICLE_POS, pos_value)
    return jsonify({"status": f"Position {position} set"})

@app.route('/calibrate/status', methods=['GET'])
async def calibration_status():
    """Returns the calibration status."""
    msg = udp_connection.recv_match(type='STATUSTEXT', blocking=True, timeout=5)
    return jsonify({"status": msg.text if msg else "No update available"})

if __name__ == '__main__':
    app.run(port=5000)
