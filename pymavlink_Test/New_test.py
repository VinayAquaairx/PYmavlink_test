import asyncio
from pymavlink import mavutil
from quart import Quart, jsonify
import time

# Initialize Quart app
app = Quart(__name__)

# Global variable to store channel values
channel_values = {}

async def connect_mavlink():
    global channel_values
    while True:
        try:
            # Connect to the radio controller
            master = mavutil.mavlink_connection('tcp:192.168.4.1:8888', autoreconnect=True, force_connected=False)

            print("Waiting for heartbeat...")
            # Wait for the first heartbeat
            await asyncio.to_thread(master.wait_heartbeat)
            print(f"Heartbeat received from system (system {master.target_system} component {master.target_component})")

            while True:
                # Read the RC channels
                msg = await asyncio.to_thread(master.recv_match, type='RC_CHANNELS', blocking=True, timeout=1)
                if msg:
                    # Update channel values
                    channel_values = {
                        f'channel_{i+1}': getattr(msg, f'chan{i+1}_raw')
                        for i in range(18)  # MAVLink supports up to 18 channels
                    }
                    print("Updated channel values:", channel_values)
                else:
                    print("No message received")
        except Exception as e:
            print(f"Connection error: {e}")
            print("Retrying in 5 seconds...")
            await asyncio.sleep(5)

@app.route('/channels')
async def get_channels():
    return jsonify(channel_values)

@app.before_serving
async def startup():
    app.add_background_task(connect_mavlink)

if __name__ == '__main__':
    app.run(host='0.0.0.0', port=5000)