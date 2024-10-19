import asyncio
import time
import logging
from pymavlink import mavutil
from quart import Quart, jsonify, request
from quart_cors import cors

logging.basicConfig(level=logging.DEBUG, format='%(asctime)s - %(levelname)s - %(message)s')
logger = logging.getLogger(__name__)

app = Quart(__name__)
app = cors(app, allow_origin="http://localhost:5173", allow_credentials=True)

drone = None

calibration_data = {
    "status_text": "",
    "heartbeat": "Disconnected",
    "compasses": [],
    "calibration_started": False
}

async def get_parameter(param_name):
    drone.mav.param_request_read_send(
        drone.target_system, drone.target_component,
        param_name.encode(), -1
    )
    start_time = time.time()
    while time.time() - start_time < 1:
        msg = drone.recv_match(type='PARAM_VALUE', blocking=False)
        if msg and msg.param_id == param_name:
            return msg.param_value
        await asyncio.sleep(0.1)
    logger.warning(f"Timeout getting parameter: {param_name}")
    return None

async def request_data_streams():
    message_rates = {
        mavutil.mavlink.MAV_DATA_STREAM_EXTENDED_STATUS: 2,
        mavutil.mavlink.MAV_DATA_STREAM_POSITION: 2,
        mavutil.mavlink.MAV_DATA_STREAM_EXTRA1: 4,
        mavutil.mavlink.MAV_DATA_STREAM_EXTRA2: 4,
        mavutil.mavlink.MAV_DATA_STREAM_EXTRA3: 2,
        mavutil.mavlink.MAV_DATA_STREAM_RAW_SENSORS: 2,
        mavutil.mavlink.MAV_DATA_STREAM_RC_CHANNELS: 2,
    }
    
    for stream_id, rate in message_rates.items():
        drone.mav.request_data_stream_send(
            drone.target_system, drone.target_component,
            stream_id, rate, 1
        )
        logger.info(f"Requested data stream {stream_id} at {rate} Hz")
        await asyncio.sleep(0.1)

async def request_autopilot_capabilities():
    drone.mav.command_long_send(
        drone.target_system, drone.target_component,
        mavutil.mavlink.MAV_CMD_REQUEST_AUTOPILOT_CAPABILITIES,
        0, 1, 0, 0, 0, 0, 0, 0
    )
    logger.info("Requested autopilot capabilities")

async def send_banner_request():
    drone.mav.command_long_send(
        drone.target_system, drone.target_component,
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

async def connect_drone():
    global drone
    try:
        # drone = mavutil.mavlink_connection('COM9', baud=9600)
        drone = mavutil.mavlink_connection('tcp:192.168.4.1:8888')
        # drone = mavutil.mavlink_connection('udp:127.0.0.1:14550')
        await asyncio.to_thread(drone.wait_heartbeat)
        logger.info(f"Heartbeat from system (system {drone.target_system} component {drone.target_component})")
        
        # Request data streams
        await request_data_streams()
        
        # Request autopilot capabilities
        await request_autopilot_capabilities()
        
        # Send banner request
        await send_banner_request()
        
        # Fetch parameter file (placeholder)
        await fetch_parameter_file()
        
        # Detect compasses
        await detect_compasses()
    except Exception as e:
        logger.error(f"Error connecting to drone: {e}")
        raise

async def send_heartbeat():
    while True:
        if drone is not None:
            drone.mav.heartbeat_send(
                mavutil.mavlink.MAV_TYPE_GCS,
                mavutil.mavlink.MAV_AUTOPILOT_INVALID,
                0, 0, 0
            )
        await asyncio.sleep(1)

@app.before_serving
async def startup():
    try:
        await connect_drone()
        asyncio.create_task(send_heartbeat())
        asyncio.create_task(update_data_continuously())
    except Exception as e:
        logger.error(f"Startup failed: {e}")
        raise





@app.route('/start_calibration', methods=['POST'])
async def start_calibration():
    try:
        data = await request.json
        compass_mask = data.get("compass_mask", 0)
        
        drone.mav.command_long_send(
            drone.target_system,
            drone.target_component,
            mavutil.mavlink.MAV_CMD_DO_START_MAG_CAL,
            0, 0, compass_mask, 1, 0, 0, 0, 0
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

@app.route('/calibration_status', methods=['GET'])
async def calibration_status():
    return jsonify(calibration_data), 200

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
    while True:
        msg = drone.recv_msg()
        if msg:
            update_calibration_data(msg)
        await asyncio.sleep(0.01)

@app.route('/cancel_calibration', methods=['POST'])
async def cancel_calibration():
    try:
        drone.mav.command_long_send(
            drone.target_system,
            drone.target_component,
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
        drone.mav.command_long_send(
            drone.target_system,
            drone.target_component,
            mavutil.mavlink.MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN,
            0, 1, 0, 0, 0, 0, 0, 0
        )
        logger.info("Drone reboot command sent")
        return jsonify({"status": "Drone is rebooting"}), 200
    except Exception as e:
        logger.error(f"Error rebooting drone: {str(e)}")
        return jsonify({"error": str(e)}), 500

if __name__ == '__main__':
    app.run(port=5003)