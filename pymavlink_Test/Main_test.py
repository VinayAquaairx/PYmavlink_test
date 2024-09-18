import serial.tools.list_ports
from pymavlink import mavutil

def list_serial_ports():
    """
    List all available COM ports.
    Returns a list of serial ports.
    """
    ports = serial.tools.list_ports.comports()
    return [port.device for port in ports]

def connect_to_serial():
    """
    Try to connect to the drone on any available COM port.
    Returns the connection object if successful.
    """
    available_ports = list_serial_ports()
    
    if not available_ports:
        print("No COM ports found.")
        return None

    for port in available_ports:
        try:
            print(f"Attempting to connect via serial on {port}...")
            connection = mavutil.mavlink_connection(port, baud=115200)
            heartbeat = connection.wait_heartbeat(timeout=10)
            if heartbeat:
                print(f"Connected via serial on {port}, heartbeat received from system {connection.target_system}, component {connection.target_component}")
                return connection
            else:
                print(f"No heartbeat received on {port}.")
        except Exception as e:
            print(f"Serial connection to {port} failed: {e}")

    print("Failed to connect via serial.")
    return None

def connect_to_tcp():
    """
    Try to connect to SITL via TCP (ArduPilot or jMAVSim in WSL).
    Returns the connection object if successful.
    """
    tcp_address = 'tcp:192.168.4.1:8888' 
    try:
        print("Attempting to connect via TCP...")
        connection = mavutil.mavlink_connection(tcp_address)
        heartbeat = connection.wait_heartbeat(timeout=10)
        if heartbeat:
            print(f"Connected via TCP, heartbeat received from system {connection.target_system}, component {connection.target_component}")
            return connection
    except Exception as e:
        print(f"TCP connection failed: {e}")
    
    return None

def connect_to_udp():
    """
    Try to connect to SITL via UDP (ArduPilot or jMAVSim in WSL).
    Returns the connection object if successful.
    """
    udp_address = 'udp:127.0.0.1:14550' 
    try:
        print("Attempting to connect via UDP...")
        connection = mavutil.mavlink_connection(udp_address)
        heartbeat = connection.wait_heartbeat(timeout=10)
        if heartbeat:
            print(f"Connected via UDP, heartbeat received from system {connection.target_system}, component {connection.target_component}")
            return connection
    except Exception as e:
        print(f"UDP connection failed: {e}")
    
    return None

def decode_firmware_version(version):
    """
    Decode the firmware version from a single integer to a readable format.
    """
    major = (version >> 24) & 0xFF
    minor = (version >> 16) & 0xFF
    patch = (version >> 8) & 0xFF
    return f"{major}.{minor}.{patch}"

def print_firmware_and_vehicle_info(connection):
    """
    Print the firmware type, vehicle type, and software version from the drone.
    """
    heartbeat = connection.recv_match(type='HEARTBEAT', blocking=True)
    
    if heartbeat:
        firmware = mavutil.mavlink.enums['MAV_AUTOPILOT'][heartbeat.autopilot].description
        vehicle = mavutil.mavlink.enums['MAV_TYPE'][heartbeat.type].description
        mavlink_version = heartbeat.mavlink_version
        
        print(f"Firmware: {firmware}")
        print(f"Vehicle Type: {vehicle}")
        print(f"MAVLink Version: {mavlink_version}")
    else:
        print("Failed to retrieve firmware and vehicle information.")


def get_statustext_messages(connection):
    """
    Retrieve and print STATUSTEXT messages from the drone.
    """
    print("Fetching STATUSTEXT messages...")
    while True:
        message = connection.recv_match(type="STATUSTEXT", blocking=True, timeout=5)
        if message:
            print(f"[{message.severity}] {message.text}")
        else:
            print("No more STATUSTEXT messages.")
            break


def get_autopilot_version_info(connection):
    """
    Retrieve and print AUTOPILOT_VERSION message information, such as
    the firmware version, board type, and other system information.
    """
    print("Requesting AUTOPILOT_VERSION...")
    connection.mav.autopilot_version_request_send(connection.target_system, connection.target_component)
    
    while True:
        message = connection.recv_match(type="AUTOPILOT_VERSION", blocking=True, timeout=5)
        if message:
            firmware_version = decode_firmware_version(message.flight_sw_version)
            print(f"Firmware Version: {firmware_version}")
            print(f"Board Version: {message.board_version}")
            print(f"Capabilities: {message.capabilities}")
            print(f"Vendor ID: {message.vendor_id}")
            print(f"Product ID: {message.product_id}")
            print(f"UID: {message.uid}")
            break
        else:
            print("No AUTOPILOT_VERSION message received.")
            break

def get_radio_status(connection):
    """
    Retrieve and print RADIO_STATUS messages from the drone.
    """
    print("Fetching RADIO_STATUS messages...")
    while True:
        message = connection.recv_match(type="RADIO_STATUS", blocking=True, timeout=5)
        if message:
            print(f"RSSI: {message.rssi}, Remote RSSI: {message.remrssi}, TxBuf: {message.txbuf}, Noise: {message.noise}, Remote Noise: {message.remnoise}")
        else:
            print("No more RADIO_STATUS messages.")
            break

def get_gps_and_battery_status(connection):
    """
    Retrieve and print GPS and battery data from the drone.
    """
    while True:
        gps_msg = connection.recv_match(type='GPS_RAW_INT', blocking=True, timeout=5)
        battery_msg = connection.recv_match(type='BATTERY_STATUS', blocking=True, timeout=5)
        
        if gps_msg:
            gps_data = {
                "latitude": gps_msg.lat / 1e7,
                "longitude": gps_msg.lon / 1e7,
                "altitude": gps_msg.alt / 1000.0,
                "satellites_visible": gps_msg.satellites_visible
            }
            print(f"GPS Data: {gps_data}")
        
        if battery_msg:
            battery_data = {
                "voltage": battery_msg.voltages[0] / 1000.0,
                "current": battery_msg.current_battery / 100.0,
                "battery_remaining": battery_msg.battery_remaining
            }
            print(f"Battery Data: {battery_data}")
        else:
            print("No more GPS or battery messages.")
            break

def main():
    # Try serial first
    connection = connect_to_serial()

    # If serial connection fails, try TCP and UDP
    
    if not connection:
        connection = connect_to_udp()
    
    if not connection:
        connection = connect_to_tcp()

    if connection:
        print("Connection successful. Ready to retrieve information.")
        
        # Print firmware and vehicle info
        print_firmware_and_vehicle_info(connection)
        
        # Retrieve and print RADIO_STATUS messages (for RSSI, noise, etc.)
        get_radio_status(connection)
        
        # Retrieve and print STATUSTEXT messages
        get_statustext_messages(connection)

        # Retrieve and print GPS and battery status
        get_gps_and_battery_status(connection)


        # Retrieve and print AUTOPILOT_VERSION information
        get_autopilot_version_info(connection)

    else:
        print("Failed to connect to the drone.")

if __name__ == "__main__":
    main()
