from quart import Quart, jsonify, request
from quart_cors import cors
from pymavlink import mavutil
import asyncio
from hypercorn.asyncio import serve
from hypercorn.config import Config

app = Quart(__name__)
app = cors(app)
connection = None

# Dictionary mapping calibration positions to their corresponding values
ACCEL_CAL_POSITIONS = {
    "level": 1,
    "left": 2,
    "right": 3,
    "nose_up": 4,
    "nose_down": 5,
    "tail_down": 6
}

# Function to establish connection with the drone
async def connect_to_drone():
    global connection
    while True:
        try:
            # Attempt to connect to the drone
            connection = mavutil.mavlink_connection("udp:127.0.0.1:14550")
            # connection = mavutil.mavlink_connection("tcp:192.168.4.1:8888")
            connection.wait_heartbeat()
            print("Connected to drone.")
            break
        except Exception as e:
            print(f"Failed to connect, retrying: {e}")
            await asyncio.sleep(5)

# Function to continuously send heartbeat messages to maintain connection
async def heartbeat_monitor():
    while True:
        if connection is not None:
            try:
                connection.mav.heartbeat_send(
                    mavutil.mavlink.MAV_TYPE_ONBOARD_CONTROLLER,
                    mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0, 0
                )
            except Exception as e:
                print(f"Heartbeat error: {e}")
                await connect_to_drone()
        await asyncio.sleep(1)

# Function to start full accelerometer calibration
async def start_full_accel_calibration():
    try:
        connection.mav.command_long_send(
            connection.target_system, connection.target_component,
            mavutil.mavlink.MAV_CMD_PREFLIGHT_CALIBRATION,
            0, 0, 0, 0, 0, 1, 0, 0
        )
        return "Full accelerometer calibration started."
    except Exception as e:
        return f"Error: {e}"

# Function to start simple accelerometer calibration
async def start_simple_accel_calibration():
    try:
        connection.mav.command_long_send(
            connection.target_system, connection.target_component,
            mavutil.mavlink.MAV_CMD_PREFLIGHT_CALIBRATION,
            0, 0, 0, 0, 0, 4, 0, 0
        )
        return "Simple accelerometer calibration started."
    except Exception as e:
        return f"Error: {e}"

# Function to calibrate level position
async def calibrate_level():
    try:
        connection.mav.command_long_send(
            connection.target_system, connection.target_component,
            mavutil.mavlink.MAV_CMD_PREFLIGHT_CALIBRATION,
            0, 0, 0, 0, 0, 2, 0, 0
        )
        return "Calibrate level initiated."
    except Exception as e:
        return f"Error: {e}"

# Function to send calibration step for a specific position
async def send_calibration_step(position):
    try:
        connection.mav.command_long_send(
            connection.target_system, connection.target_component,
            mavutil.mavlink.MAV_CMD_ACCELCAL_VEHICLE_POS,
            0, ACCEL_CAL_POSITIONS[position], 0, 0, 0, 0, 0, 0
        )
        return f"Position {position} set for accelerometer calibration."
    except Exception as e:
        return f"Error setting position: {e}"

# Route to start full calibration
@app.route('/calibrate', methods=['POST'])
async def calibrate():
    result = await start_full_accel_calibration()
    return jsonify({"message": result})

# Route to start simple calibration
@app.route('/simple_calibrate', methods=['POST'])
async def simple_calibrate():
    result = await start_simple_accel_calibration()
    return jsonify({"message": result})

# Route to calibrate level position
@app.route('/calibrate_level', methods=['POST'])
async def level_calibrate():
    result = await calibrate_level()
    return jsonify({"message": result})

# Route to set calibration position
@app.route('/set_position/<position>', methods=['POST'])
async def set_position(position):
    if position in ACCEL_CAL_POSITIONS:
        result = await send_calibration_step(position)
        return jsonify({"message": result})
    else:
        return jsonify({"error": "Invalid position"}), 400

# Route to get calibration status (replaces WebSocket)
@app.route('/calibration_status', methods=['GET'])
async def calibration_status():
    if connection is not None:
        try:
            msg = connection.recv_match(type='STATUSTEXT', blocking=True, timeout=1)
            return jsonify({"status": msg.text})
        except Exception as e:
            return jsonify({"error": f"Error during status update: {e}"})
    else:
        return jsonify({"error": "Not connected to drone"})

async def main():
    await connect_to_drone()
    asyncio.create_task(heartbeat_monitor())

    config = Config()
    config.bind = ["127.0.0.1:5001"]
    await serve(app, config)

if __name__ == "__main__":
    asyncio.run(main())