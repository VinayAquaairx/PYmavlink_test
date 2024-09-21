import React, { useState, useEffect, useCallback, useRef } from 'react';
import { MapContainer, TileLayer, Marker, Popup, useMapEvents } from 'react-leaflet';
import L from 'leaflet';
import 'leaflet/dist/leaflet.css';

const API_URL = 'http://localhost:5001';

const styles = {
  container: { fontFamily: 'Arial, sans-serif', maxWidth: '1200px', margin: '0 auto', padding: '20px' },
  header: { fontSize: '24px', fontWeight: 'bold', marginBottom: '20px' },
  card: { border: '1px solid #ddd', borderRadius: '4px', padding: '15px', marginBottom: '15px' },
  cardTitle: { fontSize: '18px', fontWeight: 'bold', marginBottom: '10px' },
  input: { width: '100%', padding: '8px', marginBottom: '10px' },
  select: { width: '100%', padding: '8px', marginBottom: '10px' },
  button: { backgroundColor: '#4CAF50', border: 'none', color: 'white', padding: '10px 20px', textAlign: 'center', textDecoration: 'none', display: 'inline-block', fontSize: '16px', margin: '4px 2px', cursor: 'pointer' },
  grid: { display: 'grid', gridTemplateColumns: 'repeat(auto-fill, minmax(250px, 1fr))', gap: '15px' },
  horizon: { width: '200px', height: '200px', border: '2px solid black', borderRadius: '50%', position: 'relative', overflow: 'hidden' },
  horizonLine: { width: '100%', height: '2px', backgroundColor: 'black', position: 'absolute', top: '50%', transform: 'translateY(-50%)' },
  compass: { width: '200px', height: '200px', border: '2px solid black', borderRadius: '50%', position: 'relative' },
  compassNeedle: { width: '2px', height: '80px', backgroundColor: 'red', position: 'absolute', top: '50%', left: '50%', transformOrigin: 'bottom center' },
  notificationBox: {
    height: '200px',
    overflowY: 'scroll',
    border: '1px solid #ccc',
    padding: '10px',
    marginBottom: '15px',
  },
};

const mapStyles = {
  height: '400px',
  width: '100%',
  marginBottom: '15px',
};

const Card = ({ children, title }) => (
  <div style={styles.card}>
    <h3 style={styles.cardTitle}>{title}</h3>
    {children}
  </div>
);

const Horizon = ({ pitch, roll }) => (
  <div style={styles.horizon}>
    <div style={{
      ...styles.horizonLine,
      transform: `translateY(-50%) rotate(${roll}deg) translateY(${pitch}px)`
    }} />
  </div>
);

const Compass = ({ heading }) => (
  <div style={styles.compass}>
    <div style={{
      ...styles.compassNeedle,
      transform: `translateX(-50%) rotate(${heading}deg)`
    }} />
  </div>
);

const DroneControl = () => {
  const [status, setStatus] = useState({});
  const [comPorts, setComPorts] = useState([]);
  const [selectedPort, setSelectedPort] = useState('');
  const [baudRate, setBaudRate] = useState(115200);
  const [connectionType, setConnectionType] = useState('serial');
  const [ipAddress, setIpAddress] = useState('127.0.0.1');
  const [port, setPort] = useState(14550);
  const [isConnected, setIsConnected] = useState(false);
  const [takeoffAltitude, setTakeoffAltitude] = useState(10);
  const [homePosition, setHomePosition] = useState({ lat: 0, lon: 0, alt: 0 });
  const [mapCenter, setMapCenter] = useState([0, 0]);
  const [dronePosition, setDronePosition] = useState(null);
  const [selectedPosition, setSelectedPosition] = useState(null);
  const mapRef = useRef(null);
  const [commandStatus, setCommandStatus] = useState('');
  const [messages, setMessages] = useState([]);

  const fetchStatus = useCallback(async () => {
    try {
      const response = await fetch(`${API_URL}/status`);
      const data = await response.json();
      setStatus(data);
    } catch (error) {
      console.error('Error fetching status:', error);
    }
  }, []);

  useEffect(() => {
    const fetchComPorts = async () => {
      try {
        const response = await fetch(`${API_URL}/com_ports`);
        const data = await response.json();
        setComPorts(data.ports);
      } catch (error) {
        console.error('Error fetching COM ports:', error);
      }
    };

    fetchComPorts();
  }, []);

  const fetchMessages = useCallback(async () => {
    try {
      const response = await fetch(`${API_URL}/messages`);
      const newMessages = await response.json();
      setMessages(prevMessages => [...prevMessages, ...newMessages]);
    } catch (error) {
      console.error('Error fetching messages:', error);
    }
  }, []);

  useEffect(() => {
    const interval = setInterval(fetchMessages, 1000);
    return () => clearInterval(interval);
  }, [fetchMessages]);

  const MapEvents = () => {
    useMapEvents({
      click(e) {
        setSelectedPosition([e.latlng.lat, e.latlng.lng]);
      },
    });
    return null;
  };

  useEffect(() => {
    if (status.gps) {
      setDronePosition([status.gps.lat, status.gps.lon]);
      if (!mapCenter[0] && !mapCenter[1]) {
        setMapCenter([status.gps.lat, status.gps.lon]);
        mapRef.current?.setView([status.gps.lat, status.gps.lon], 15);
      }
    }
  }, [status.gps, mapCenter]);

  useEffect(() => {
    if (isConnected) {
      const interval = setInterval(fetchStatus, 100); // Fetch every 100ms
      return () => clearInterval(interval);
    }
  }, [isConnected, fetchStatus]);

  const handleConnect = async () => {
    const connectData = {
      protocol: connectionType,
      port: connectionType === 'serial' ? selectedPort : port,
      baudrate: parseInt(baudRate),
      ip: ipAddress,
    };

    try {
      const response = await fetch(`${API_URL}/connect`, {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify(connectData),
      });

      if (response.ok) {
        setIsConnected(true);
      } else {
        console.error('Failed to connect');
      }
    } catch (error) {
      console.error('Error connecting:', error);
    }
  };

  const handleDisconnect = async () => {
    try {
      const response = await fetch(`${API_URL}/disconnect`, { method: 'POST' });
      if (response.ok) {
        setIsConnected(false);
      } else {
        console.error('Failed to disconnect');
      }
    } catch (error) {
      console.error('Error disconnecting:', error);
    }
  };

  const sendCommand = async (command, params = {}) => {
    try {
      setCommandStatus('Sending command...');
      const response = await fetch(`${API_URL}/command`, {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ command, ...params }),
      });
      const data = await response.json();
      if (response.ok) {
        setCommandStatus(`Command ${command} executed successfully`);
      } else {
        setCommandStatus(`Error: ${data.status}`);
      }
    } catch (error) {
      console.error(`Error sending ${command} command:`, error);
      setCommandStatus(`Error sending ${command} command`);
    }
  };

  const handleSetHome = () => {
    if (selectedPosition) {
      sendCommand('set_home', { 
        latitude: selectedPosition[0], 
        longitude: selectedPosition[1], 
        altitude: homePosition.alt || 0 
      });
    } else {
      setCommandStatus('Please select a position on the map first');
    }
  };

  const handleGoTo = () => {
    if (selectedPosition) {
      const altitude = prompt("Enter altitude for Go To command (in meters):");
      if (altitude !== null && !isNaN(parseFloat(altitude))) {
        sendCommand('goto', { 
          latitude: selectedPosition[0], 
          longitude: selectedPosition[1], 
          altitude: parseFloat(altitude) 
        });
      } else {
        setCommandStatus('Invalid altitude entered');
      }
    } else {
      setCommandStatus('Please select a position on the map first');
    }
  };

  const handleArm = () => sendCommand('arm');

  const handleDisarm = () => sendCommand('disarm');

  const handleTakeoff = () => {
    if (takeoffAltitude > 0) {
      sendCommand('takeoff', { altitude: takeoffAltitude });
    } else {
      setCommandStatus('Please set a valid takeoff altitude');
    }
  };

  const handleLand = () => sendCommand('land');

  const handleRTL = () => sendCommand('rtl');



  
  return (
    <div style={styles.container}>
      <h1 style={styles.header}>Drone Control Panel</h1>
      
      <Card title="Connection Settings">
        <select style={styles.select} value={connectionType} onChange={(e) => setConnectionType(e.target.value)}>
          <option value="serial">Serial</option>
          <option value="udp">UDP</option>
          <option value="tcp">TCP</option>
        </select>
        
        {connectionType === 'serial' ? (
          <>
            <select style={styles.select} value={selectedPort} onChange={(e) => setSelectedPort(e.target.value)}>
              {comPorts.map((port) => (
                <option key={port} value={port}>{port}</option>
              ))}
            </select>
            <input
              style={styles.input}
              type="number"
              value={baudRate}
              onChange={(e) => setBaudRate(e.target.value)}
              placeholder="Baud Rate"
            />
          </>
        ) : (
          <>
            <input
              style={styles.input}
              type="text"
              value={ipAddress}
              onChange={(e) => setIpAddress(e.target.value)}
              placeholder="IP Address"
            />
            <input
              style={styles.input}
              type="number"
              value={port}
              onChange={(e) => setPort(e.target.value)}
              placeholder="Port"
            />
          </>
        )}
        
        <button style={styles.button} onClick={isConnected ? handleDisconnect : handleConnect}>
          {isConnected ? 'Disconnect' : 'Connect'}
        </button>
      </Card>

      <Card title="Control Commands">
        <button style={styles.button} onClick={handleArm}>Arm</button>
        <button style={styles.button} onClick={handleDisarm}>Disarm</button>

        <input
          style={styles.input}
          type="number"
          value={takeoffAltitude}
          onChange={(e) => setTakeoffAltitude(Number(e.target.value))}
          placeholder="Takeoff Altitude (m)"
        />

        <button style={styles.button} onClick={handleTakeoff}>Takeoff</button>
        <button style={styles.button} onClick={handleLand}>Land</button>
        
        <button style={styles.button} onClick={handleRTL}>RTL</button>
        <button style={styles.button} onClick={handleSetHome}>Set Home</button>
        <button style={styles.button} onClick={handleGoTo}>Go To Selected</button>
        <div>Command Status: {commandStatus}</div>
      </Card>



      <Card title="Notifications">
        <div style={styles.notificationBox}>
          {messages.map((message, index) => (
            <div key={index}>{message}</div>
          ))}
        </div>
      </Card>



      <div style={styles.grid}>
        <Card title="Connection Status">
          <div>Status: {status.connection_status}</div>
        </Card>

        <Card title="System Info">
          <div>System ID: {status.system_id}</div>
          <div>Component ID: {status.component_id}</div>
          <div>Firmware Type: {status.firmware_type}</div>
          <div>Vehicle Type: {status.vehicle_type}</div>
          <div>Firmware Version: {status.firmware_version}</div>
          <div>Board Version: {status.board_version}</div>
        </Card>

        <Card title="Battery">
          <div>Remaining: {status.battery_status?.battery_remaining}%</div>
          <div>Voltage: {status.battery_status?.voltage}V</div>
          <div>Current: {status.battery_status?.current}A</div>
          <div>Consumed: {status.battery_status?.capacity_consumed} mAh</div>
        </Card>

        <Card title="GPS">
          <div>Lat: {status.gps?.lat}</div>
          <div>Lon: {status.gps?.lon}</div>
          <div>Alt: {status.gps?.alt}m</div>
          <div>Relative Alt: {status.gps?.relative_alt}m</div>
          <div>Satellites: {status.satellite_count}</div>
          <div>HDOP: {status.hdop}</div>
          <div>Fix Type: {status.gps_fix_type}</div>
        </Card>

        <Card title="Attitude">
          <Horizon pitch={status.attitude?.pitch} roll={status.attitude?.roll} />
          <div>Roll: {status.attitude?.roll}°</div>
          <div>Pitch: {status.attitude?.pitch}°</div>
          <div>Yaw: {status.attitude?.yaw}°</div>
        </Card>

        <Card title="Heading">
          <Compass heading={status.heading} />
          <div>Heading: {status.heading}°</div>
        </Card>

        <Card title="Speed">
          <div>Ground Speed: {status.groundspeed} m/s</div>
          <div>Air Speed: {status.airspeed} m/s</div>
        </Card>

        <Card title="Home">
          <div>Distance: {status.distance_to_home} m</div>
        </Card>

        <Card title="Radio">
          <div>RSSI: {status.rssi_dBm} dBm</div>
          <div>Remote RSSI: {status.remrssi_dBm} dBm</div>
          <div>RX Errors: {status.rx_errors}</div>
          <div>TX Buffer: {status.tx_buffer}</div>
          <div>Local Noise: {status.local_noise}</div>
          <div>Remote Noise: {status.remote_noise}</div>
        </Card>

        <Card title="Rangefinder">
          <div>Distance: {status.rangefinder?.distance} m</div>
          <div>Min Distance: {status.rangefinder?.min_distance} m</div>
          <div>Max Distance: {status.rangefinder?.max_distance} m</div>
        </Card>

        <Card title="Flight Status">
          <div>Armed: {status.armed ? 'Yes' : 'No'}</div>
          <div>MAV Mode: {status.mav_mode}</div>
          <div>Landed State: {status.mav_landed_state}</div>
        </Card>

        <Card title="Channel Outputs">
          {Object.entries(status.channel_outputs || {}).map(([channel, value]) => (
            <div key={channel}>{channel}: {value}</div>
          ))}
        </Card>
        <div style={styles.container}>

    
    <Card title="Set Home Position">
      <input
        style={styles.input}
        type="number"
        value={homePosition.lat}
        onChange={(e) => setHomePosition({ ...homePosition, lat: Number(e.target.value) })}
        placeholder="Latitude"
      />
      <input
        style={styles.input}
        type="number"
        value={homePosition.lon}
        onChange={(e) => setHomePosition({ ...homePosition, lon: Number(e.target.value) })}
        placeholder="Longitude"
      />
      <input
        style={styles.input}
        type="number"
        value={homePosition.alt}
        onChange={(e) => setHomePosition({ ...homePosition, alt: Number(e.target.value) })}
        placeholder="Altitude"
      />
      <button style={styles.button} onClick={handleSetHome}>Set Home Position</button>
    </Card>
    
    <Card title="Map">
        <MapContainer center={mapCenter} zoom={15} style={mapStyles} ref={mapRef}>
          <TileLayer url="https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png" />
          <MapEvents />
          {dronePosition && (
            <Marker position={dronePosition}>
              <Popup>Drone Position</Popup>
            </Marker>
          )}
          {homePosition && (
            <Marker position={homePosition}>
              <Popup>Home Position</Popup>
            </Marker>
          )}
          {selectedPosition && (
            <Marker position={selectedPosition}>
              <Popup>
                Selected Position
                <br />
                Lat: {selectedPosition[0].toFixed(6)}
                <br />
                Lon: {selectedPosition[1].toFixed(6)}
              </Popup>
            </Marker>
          )}
        </MapContainer>
      </Card>
  </div>
  
      </div>
    </div>
  );
};

export default DroneControl;