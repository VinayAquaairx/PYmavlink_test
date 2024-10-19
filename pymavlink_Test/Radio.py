import asyncio
from pymavlink import mavutil
from quart import Quart, jsonify, request
import logging
import time
from quart_cors import cors
from collections import deque

logging.basicConfig(level=logging.DEBUG)
logger = logging.getLogger(__name__)

app = Quart(__name__)
app = cors(app, allow_origin="*")

# Global variables
all_parameters = {}
rc_parameters = {}
channel_values = {}
connection_status = {"connected": False, "last_heartbeat": 0, "system_id": None, "component_id": None}
HEARTBEAT_TIMEOUT = 2
calibration_state = {
    "in_progress": False,
    "current_step": "",
    "instructions": "",
    "channel_data": {}
}
master = None
status_messages = deque(maxlen=50)

def decode_if_needed(value):
    if isinstance(value, bytes):
        return value.decode()
    return value

async def connect_mavlink():
    global master, connection_status
    # connection_string = 'udp:127.0.0.1:14550'
    connection_string = 'tcp:192.168.4.1:8888'
    while True:
        try:
            if not connection_status["connected"] or master is None:
                logger.info(f"Attempting to connect using {connection_string}")
                master = mavutil.mavlink_connection(connection_string, source_system=255, source_component=0, retries=3)
                await asyncio.to_thread(master.wait_heartbeat, timeout=10)
                connection_status["system_id"] = master.target_system
                connection_status["component_id"] = master.target_component
                logger.info(f"Heartbeat received from system {connection_status['system_id']} component {connection_status['component_id']}")
                connection_status["connected"] = True
                connection_status["last_heartbeat"] = time.time()
                master.mav.request_data_stream_send(
                    master.target_system, master.target_component,
                    mavutil.mavlink.MAV_DATA_STREAM_ALL, 10, 1
                )
                await fetch_all_parameters()

            await message_loop()
        except Exception as e:
            logger.error(f"Connection error: {e}")
            connection_status["connected"] = False
            master = None
            await asyncio.sleep(5)

async def fetch_all_parameters():
    global all_parameters, rc_parameters
    logger.info("Fetching all parameters...")
    master.mav.param_request_list_send(master.target_system, master.target_component)
    
    start_time = time.time()
    while time.time() - start_time < 30:  # 30-second timeout
        msg = await asyncio.to_thread(master.recv_match, type='PARAM_VALUE', blocking=True, timeout=1)
        if msg:
            param_id = decode_if_needed(msg.param_id)
            all_parameters[param_id] = msg.param_value
            if param_id.startswith('RC') and (param_id.endswith('_MIN') or param_id.endswith('_MAX') or param_id.endswith('_TRIM')):
                rc_parameters[param_id] = msg.param_value
        else:
            break  # No more parameters
    
    logger.info(f"Fetched {len(all_parameters)} parameters, including {len(rc_parameters)} RC parameters")
    
    # Check for missing RC parameters and request them individually
    for i in range(1, 17):
        for suffix in ['_MIN', '_MAX', '_TRIM']:
            param_name = f'RC{i}{suffix}'
            if param_name not in rc_parameters:
                await request_param(param_name)

async def request_param(param_name):
    master.mav.param_request_read_send(
        master.target_system, master.target_component,
        param_name.encode(), -1
    )
    
    start_time = time.time()
    while time.time() - start_time < 5:  # 5-second timeout
        msg = await asyncio.to_thread(master.recv_match, type='PARAM_VALUE', blocking=True, timeout=1)
        if msg and decode_if_needed(msg.param_id) == param_name:
            rc_parameters[param_name] = msg.param_value
            all_parameters[param_name] = msg.param_value
            logger.info(f"Fetched missing parameter: {param_name} = {msg.param_value}")
            return
    
    logger.warning(f"Timeout fetching {param_name}")

async def message_loop():
    global channel_values, connection_status, status_messages
    last_heartbeat_request = 0
    while connection_status["connected"]:
        try:
            msg = await asyncio.to_thread(master.recv_match, blocking=True, timeout=0.5)
            if msg:
                if msg.get_type() == 'HEARTBEAT':
                    connection_status["last_heartbeat"] = time.time()
                    connection_status["system_id"] = msg.get_srcSystem()
                    connection_status["component_id"] = msg.get_srcComponent()
                elif msg.get_type() == 'RC_CHANNELS':
                    for i in range(16):
                        channel = f'channel_{i+1}'
                        value = getattr(msg, f'chan{i+1}_raw', None)
                        channel_values[channel] = value
                        if calibration_state["in_progress"] and value is not None:
                            calibration_state["channel_data"][channel] = calibration_state["channel_data"].get(channel, []) + [value]
                elif msg.get_type() == 'STATUSTEXT':
                    status_messages.append(f"{time.strftime('%H:%M:%S')}: {decode_if_needed(msg.text)}")
                    logger.info(f"Status message: {decode_if_needed(msg.text)}")
            
            # Request heartbeat if we haven't received one in a while
            current_time = time.time()
            if current_time - connection_status["last_heartbeat"] > HEARTBEAT_TIMEOUT:
                if current_time - last_heartbeat_request > 1:  # Don't spam requests
                    master.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_GCS, mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0, 0)
                    last_heartbeat_request = current_time
                    logger.debug("Sent heartbeat request")
            
            if current_time - connection_status["last_heartbeat"] > HEARTBEAT_TIMEOUT * 2:
                logger.warning("Heartbeat timeout, reconnecting...")
                raise Exception("Heartbeat timeout")
        
        except Exception as e:
            logger.error(f"Error in message loop: {e}")
            connection_status["connected"] = False
            break

async def start_calibration():
    global calibration_state
    if not connection_status["connected"] or master is None:
        return {"error": "Not connected to the drone"}, 400

    calibration_state["in_progress"] = True
    calibration_state["current_step"] = "Move all sticks"
    calibration_state["instructions"] = "Please move all sticks and switches to their extreme positions several times."
    calibration_state["channel_data"] = {}
    
    return {"status": "Calibration started successfully"}, 200

async def save_calibration():
    global calibration_state, rc_parameters
    if not calibration_state["in_progress"]:
        return {"error": "No calibration in progress"}, 400

    try:
        new_params = {}
        for channel, data in calibration_state["channel_data"].items():
            channel_num = int(channel.split('_')[1])
            min_value = min(data)
            max_value = max(data)
            trim_value = sum(data) // len(data)  # average as trim

            new_params[f'RC{channel_num}_MIN'] = min_value
            new_params[f'RC{channel_num}_MAX'] = max_value
            new_params[f'RC{channel_num}_TRIM'] = trim_value

        # Send all parameters at once
        for param_name, value in new_params.items():
            master.mav.param_set_send(
                master.target_system, master.target_component,
                param_name.encode('utf-8'),
                float(value),
                mavutil.mavlink.MAV_PARAM_TYPE_REAL32
            )

        # Wait for confirmation
        start_time = time.time()
        confirmed_params = set()
        while time.time() - start_time < 10 and len(confirmed_params) < len(new_params):
            msg = await asyncio.to_thread(master.recv_match, type='PARAM_VALUE', blocking=True, timeout=0.1)
            if msg:
                param_id = decode_if_needed(msg.param_id)
                if param_id in new_params:
                    confirmed_params.add(param_id)
                    rc_parameters[param_id] = msg.param_value
                    logger.info(f"Parameter {param_id} confirmed set to {msg.param_value}")

        if len(confirmed_params) < len(new_params):
            logger.warning(f"Timeout: Only {len(confirmed_params)} out of {len(new_params)} parameters were confirmed")

        calibration_state["in_progress"] = False
        calibration_state["current_step"] = "Calibration complete"
        calibration_state["instructions"] = "Calibration completed successfully."
        return {"status": "Calibration saved successfully", "confirmed_params": list(confirmed_params)}, 200

    except Exception as e:
        logger.error(f"Error saving calibration: {str(e)}")
        calibration_state["in_progress"] = False
        calibration_state["current_step"] = f"Calibration error: {str(e)}"
        calibration_state["instructions"] = "An error occurred while saving calibration. Please try again."
        return {"error": str(e)}, 500

@app.route('/data', methods=['GET'])
async def get_data():
    return jsonify({
        "channel_values": channel_values,
        "rc_parameters": rc_parameters,
        "connection_status": connection_status,
        "calibration_state": calibration_state,
        "status_messages": list(status_messages)
    })

@app.route('/start_calibration', methods=['POST'])
async def initiate_calibration():
    if calibration_state["in_progress"]:
        return jsonify({"error": "Calibration already in progress"}), 400

    result, status_code = await start_calibration()
    return jsonify(result), status_code

@app.route('/save_calibration', methods=['POST'])
async def finish_calibration():
    result, status_code = await save_calibration()
    return jsonify(result), status_code

@app.before_serving
async def startup():
    app.add_background_task(connect_mavlink)

if __name__ == '__main__':
    app.run(host='0.0.0.0', port=5002)