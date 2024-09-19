import React, { useState, useEffect, useRef } from 'react';
import axios from 'axios';
import { MapContainer, TileLayer, Marker, Popup, useMapEvents } from 'react-leaflet';
import L from 'leaflet';
import 'leaflet/dist/leaflet.css';
import { PiDroneBold } from "react-icons/pi";
import { IoIosHome } from "react-icons/io";
import { FaLocationDot } from "react-icons/fa6";
import './DroneControlPanel.css';
import './App_trail.css'

// Custom icon for drone
const droneIcon = new L.Icon({
  iconUrl: PiDroneBold,
  iconRetinaUrl: PiDroneBold,
  iconSize: [25, 25],
  iconAnchor: [12, 12],
});

// Custom icon for home location
const homeIcon = new L.Icon({
  iconUrl: IoIosHome,
  iconRetinaUrl: IoIosHome,
  iconSize: [25, 25],
  iconAnchor: [12, 12],
});

// Custom icon for waypoints
const waypointIcon = new L.Icon({
  iconUrl: FaLocationDot,
  iconRetinaUrl: FaLocationDot,
  iconSize: [25, 25],
  iconAnchor: [12, 12],
  iconColor: 'red',
});

const Button = ({ onClick, className, children }) => (
  <button onClick={onClick} className={`button ${className}`}>
    {children}
  </button>
);

const Input = ({ type, value, onChange, placeholder }) => (
  <input
    type={type}
    value={value}
    onChange={onChange}
    placeholder={placeholder}
    className="input"
  />
);

const missionItemTypes = {
    WAYPOINT: { 
      name: 'Waypoint',
      params: ['Delay', 'Acceptance Radius', 'Pass Radius', 'Yaw Angle']
    },
    LOITER_UNLIMITED: { 
      name: 'Loiter Unlimited',
      params: ['Radius', 'Yaw', 'Direction']
    },
    LOITER_TIME: { 
      name: 'Loiter Time',
      params: ['Time', 'Radius', 'Yaw', 'Direction']
    },
    LOITER_TURNS: { 
      name: 'Loiter Turns',
      params: ['Turns', 'Radius', 'Yaw', 'Direction']
    },
    RETURN_TO_LAUNCH: { 
      name: 'Return to Launch',
      params: []
    },
    LAND: { 
      name: 'Land',
      params: ['Abort Alt', 'Precision Land']
    },
    TAKEOFF: { 
      name: 'Takeoff',
      params: ['Pitch Angle', 'Yaw Angle']
    },
    DO_CHANGE_SPEED: { 
      name: 'Change Speed',
      params: ['Speed Type', 'Speed', 'Throttle']
    },
    DO_SET_CAM_TRIGG_DIST: { 
      name: 'Camera Trigger Distance',
      params: ['Distance', 'Shutter', 'Trigger']
    },
    DO_SET_ROI: { 
      name: 'Region of Interest',
      params: ['ROI Mode', 'WP Index', 'ROI Index']
    },
  };

const Modal = ({ isOpen, onClose, title, children }) => {
  if (!isOpen) return null;

  return (
    <div className="modal-overlay" onClick={onClose}>
      <div className="modal" onClick={e => e.stopPropagation()}>
        <h2>{title}</h2>
        {children}
      </div>
    </div>
  );
};

const Alert = ({ children }) => (
  <div className="alert">
    {children}
  </div>
);




const ParameterList = ({ isOpen, onClose }) => {
  const [parameters, setParameters] = useState([]);
  const [loading, setLoading] = useState(true);

  useEffect(() => {
    if (isOpen) {
      fetchParameters();
    }
  }, [isOpen]);

  const fetchParameters = async () => {
    try {
      const response = await axios.get('http://localhost:5001/full_parameters');
      // Sort the parameters alphabetically by name
      const sortedParameters = response.data.parameters.sort((a, b) => 
        a.name.localeCompare(b.name)
      );
      setParameters(sortedParameters);
      setLoading(false);
    } catch (error) {
      console.error('Error fetching parameters:', error);
      setLoading(false);
    }
  };

  if (!isOpen) return null;

  return (
    <div className="parameter-list-modal">
      <div className="parameter-list-content">
        <h2>Full Parameter List</h2>
        {loading ? (
          <p>Loading parameters...</p>
        ) : (
          <table className="parameter-table">
            <thead>
              <tr>
                <th>Parameter Name</th>
                <th>Value</th>
                <th>Type</th>
              </tr>
            </thead>
            <tbody>
              {parameters.map((param, index) => (
                <tr key={index}>
                  <td>{param.name}</td>
                  <td>{param.value}</td>
                  <td>{param.type}</td>
                </tr>
              ))}
            </tbody>
          </table>
        )}
        <button className="close-button" onClick={onClose}>Close</button>
      </div>
    </div>
  );
};

const DroneControlPanel = () => {
  const [droneStatus, setDroneStatus] = useState({});
  const [isArmed, setIsArmed] = useState(false);
  const [showAltitudeModal, setShowAltitudeModal] = useState(false);
  const [altitude, setAltitude] = useState('');
  const [alertMessage, setAlertMessage] = useState('');
  const [modeName, setModeName] = useState('');
  const [dronePosition, setDronePosition] = useState([0, 0]);
  const [homePosition, setHomePosition] = useState(null);
  const [tempMarker, setTempMarker] = useState(null);
  const [waypoints, setWaypoints] = useState([]);
  const [missionMode, setMissionMode] = useState(false);
  const [selectedMode, setSelectedMode] = useState('');
  const [showParameterList, setShowParameterList] = useState(false);


  const mapRef = useRef();

  const handleCheckFullParams = () => {
    setShowParameterList(true);
  };




  const modeMap = {
    0: 'STABILIZE',
    1: 'ACRO',
    2: 'ALT_HOLD',
    3: 'AUTO',
    4: 'GUIDED',
    5: 'LOITER',
    6: 'RTL',
    7: 'CIRCLE',
    8: 'POSITION',
    9: 'LAND',
    10: 'OF_LOITER',
    11: 'DRIFT',
    13: 'SPORT',
    14: 'FLIP',
    15: 'AUTOTUNE',
    16: 'POSHOLD',
    17: 'BRAKE',
    18: 'THROW',
    19: 'AVOID_ADSB',
    20: 'GUIDED_NOGPS',
    21: 'SMART_RTL',
    22: 'FLOWHOLD',
    23: 'FOLLOW',
    24: 'ZIGZAG',
    25: 'SYSTEMID',
    26: 'AUTOROTATE',
    27: 'AUTO_RTL',
  };


  const handleModeChange = async (event) => {
    const newMode = event.target.value;
    setSelectedMode(newMode);
    await handleCommand('set_mode', { mode: newMode });
  };


  useEffect(() => {
    const fetchStatus = async () => {
      try {
        const response = await axios.get('http://localhost:5001/status');
        setDroneStatus(response.data);
        setIsArmed(response.data.armed === 128);
        setModeName(modeMap[response.data.mav_mode] || 'Unknown');
        if (response.data.gps) {
          setDronePosition([response.data.gps.lat, response.data.gps.lon]);
        }
      } catch (error) {
        console.error('Error fetching drone status:', error);
      }
    };

    fetchStatus();
    const intervalId = setInterval(fetchStatus, 1000);

    return () => clearInterval(intervalId);
  }, []);


  useEffect(() => {
    const fetchStatus = async () => {
      try {
        const response = await axios.get('http://localhost:5001/status');
        setDroneStatus(response.data);
        setIsArmed(response.data.armed === 128);
        const currentMode = modeMap[response.data.mav_mode] || 'Unknown';
        setModeName(currentMode);
        setSelectedMode(currentMode); 
        if (response.data.gps) {
          setDronePosition([response.data.gps.lat, response.data.gps.lon]);
        }
      } catch (error) {
        console.error('Error fetching drone status:', error);
      }
    };
  
    fetchStatus();
    const intervalId = setInterval(fetchStatus, 1000);
  
    return () => clearInterval(intervalId);
  }, []);



  const handleArmDisarm = () => {
    handleCommand(isArmed ? 'disarm' : 'arm');
  };

  const handleTakeoff = async () => {
    if (modeName !== 'GUIDED') {
      const confirmGuidedMode = window.confirm('Drone is not in GUIDED mode. Switch to GUIDED mode?');
      if (confirmGuidedMode) {
        await handleCommand('set_mode', { mode: 'GUIDED' });
      } else {
        return;
      }
    }

    if (!isArmed) {
      const confirmArm = window.confirm('Drone is not armed. Arm the drone?');
      if (confirmArm) {
        await handleCommand('arm');
      } else {
        return;
      }
    }

    setShowAltitudeModal(true);
  };

  const handleAltitudeSubmit = () => {
    setShowAltitudeModal(false);
    handleCommand('takeoff', { altitude: parseFloat(altitude) });
  };

  const handleSetHomeLocation = () => {
    if (tempMarker) {
      handleCommand('set_home', {
        latitude: tempMarker.lat,
        longitude: tempMarker.lng,
        altitude: droneStatus.gps ? droneStatus.gps.alt : 0
      });
      setHomePosition(tempMarker);
      setTempMarker(null);
    }
  };

  const handleAddWaypoint = (latlng) => {
    const newWaypoint = {
      id: Date.now(),
      lat: latlng.lat,
      lng: latlng.lng,
      altitude: 10,
      type: 'WAYPOINT',
      params: {
        Delay: 0,
        'Acceptance Radius': 5,
        'Pass Radius': 0,
        'Yaw Angle': 0
      }
    };
    setWaypoints([...waypoints, newWaypoint]);
  };

  const handleUpdateWaypoint = (index, field, value) => {
    const newWaypoints = [...waypoints];
    if (field === 'type') {
      newWaypoints[index] = {
        ...newWaypoints[index],
        type: value,
        params: Object.fromEntries(missionItemTypes[value].params.map(param => [param, 0]))
      };
    } else if (field.startsWith('param_')) {
      const paramName = field.split('param_')[1];
      newWaypoints[index].params[paramName] = value;
    } else {
      newWaypoints[index][field] = value;
    }
    setWaypoints(newWaypoints);
  };

  const renderWaypointParams = (waypoint, index) => {
    return missionItemTypes[waypoint.type].params.map(param => (
      <div key={param}>
        {param}:
        <input
          type="number"
          value={waypoint.params[param]}
          onChange={(e) => handleUpdateWaypoint(index, `param_${param}`, parseFloat(e.target.value))}
        />
      </div>
    ));
  };

    const handleStartMission = async () => {
    try {
      const response = await axios.post('http://localhost:5001/mission', { waypoints });
      if (response.data.status === 'Mission started successfully') {
        setAlertMessage('Mission started successfully');
      } else {
        setAlertMessage(`Failed to start mission: ${response.data.status}`);
      }
    } catch (error) {
      console.error('Error starting mission:', error);
      setAlertMessage(`Error starting mission: ${error.response?.data?.status || error.message}`);
    }
  };

  const handleCommand = async (command, params = {}) => {
    try {
      const response = await axios.post('http://localhost:5001/command', { command, ...params });
      if (response.data.status === 'success') {
        setAlertMessage(`${command} command executed successfully`);
      } else {
        setAlertMessage(`Failed to execute ${command} command: ${response.data.status}`);
      }
    } catch (error) {
      console.error(`Error executing ${command} command:`, error);
      setAlertMessage(`Error executing ${command} command: ${error.response?.data?.status || error.message}`);
    }
  };

  const MapEvents = () => {
    useMapEvents({
      click(e) {
        if (missionMode) {
          handleAddWaypoint(e.latlng);
        } else {
          setTempMarker(e.latlng);
        }
      },
    });
    return null;
  };

  return (
    <div className="drone-control-panel">
      <h1>Drone Control Panel</h1>
      
      <div className="status-summary">
        <p>Mode: {modeName}</p>
        <p>Armed: {isArmed ? 'Yes' : 'No'}</p>
      </div>

      <Button 
        onClick={handleArmDisarm}
        className={isArmed ? 'disarm-button' : 'arm-button'}
      >
        {isArmed ? 'Disarm' : 'Arm'}
      </Button>

      <Button 
        onClick={handleTakeoff}
        className="takeoff-button"
      >
        Takeoff
      </Button>

      <Button 
        onClick={() => handleCommand('land')}
        className="land-button"
      >
        Land
      </Button>

      <Button 
        onClick={() => handleCommand('rtl')}
        className="rtl-button"
      >
        Return to Launch
      </Button>
    <button 
        onClick={handleCheckFullParams}
        className="check-params-button"
      >
        Check Full Parameter List
      </button>

      <ParameterList 
        isOpen={showParameterList} 
        onClose={() => setShowParameterList(false)} 
      />
      <div className="mode-selector">
        <label htmlFor="mode-select">Flight Mode: </label>
        <select
          id="mode-select"
          value={selectedMode}
          onChange={handleModeChange}
          className="mode-dropdown"
        >
          <option value="">Select Mode</option>
          {Object.entries(modeMap).map(([key, mode]) => (
            <option key={key} value={mode}>
              {mode}
            </option>
          ))}
        </select>
      </div>

      <Button
        onClick={() => setMissionMode(!missionMode)}
        className={missionMode ? 'mission-active' : 'mission-inactive'}
      >
        {missionMode ? 'Exit Mission Mode' : 'Enter Mission Mode'}
      </Button>

      {missionMode && (
        <div className="mission-controls">
          <Button
            onClick={handleStartMission}
            className="start-mission-button"
          >
            Start Mission
          </Button>
          <Button
            onClick={() => setWaypoints([])}
            className="clear-mission-button"
          >
            Clear Mission
          </Button>
        </div>
      )}

      {alertMessage && (
        <Alert>
          {alertMessage}
        </Alert>
      )}

      <Modal
        isOpen={showAltitudeModal}
        onClose={() => setShowAltitudeModal(false)}
        title="Enter Takeoff Altitude"
      >
        <Input
          type="number"
          value={altitude}
          onChange={(e) => setAltitude(e.target.value)}
          placeholder="Altitude in meters"
        />
        <Button onClick={handleAltitudeSubmit} className="submit-button">
          Submit
        </Button>
      </Modal>

      <div className="map-container">
        <MapContainer
          center={dronePosition}
          zoom={13}
          style={{ height: '400px', width: '100%' }}
          ref={mapRef}
        >
          <TileLayer
            url="https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png"
            attribution='&copy; <a href="https://www.openstreetmap.org/copyright">OpenStreetMap</a> contributors'
          />
          <MapEvents />
          <Marker position={dronePosition} icon={droneIcon}>
            <Popup>Drone Location</Popup>
          </Marker>
          {homePosition && (
            <Marker position={homePosition} icon={homeIcon}>
              <Popup>Home Location</Popup>
            </Marker>
          )}
          {tempMarker && (
            <Marker position={tempMarker}>
              <Popup>
                <Button onClick={handleSetHomeLocation}>Set as Home</Button>
              </Popup>
            </Marker>
          )}
          {waypoints.map((waypoint, index) => (
            <Marker
              key={waypoint.id}
              position={[waypoint.lat, waypoint.lng]}
              icon={waypointIcon}
            >
              <Popup>
                <div>Waypoint {index + 1}</div>
                <div>Lat: {waypoint.lat.toFixed(6)}</div>
                <div>Lon: {waypoint.lng.toFixed(6)}</div>
                <div>
                  Alt: 
                  <input
                    type="number"
                    value={waypoint.altitude}
                    onChange={(e) => handleUpdateWaypoint(index, 'altitude', parseFloat(e.target.value))}
                  />
                </div>
                <div>
                  Type: 
                  <select
                    value={waypoint.type}
                    onChange={(e) => handleUpdateWaypoint(index, 'type', e.target.value)}
                  >
                    {Object.entries(missionItemTypes).map(([type, details]) => (
                      <option key={type} value={type}>{details.name}</option>
                    ))}
                  </select>
                </div>
                {renderWaypointParams(waypoint, index)}
                <div>
                  Distance: {calculateDistance(
                    index === 0 ? dronePosition : waypoints[index-1],
                    waypoint
                  ).toFixed(2)} m
                </div>
              </Popup>
            </Marker>
          ))}
        </MapContainer>
      </div>
      
      <div className="waypoint-list">
        <h3>Mission Items</h3>
        {waypoints.map((waypoint, index) => (
          <div key={waypoint.id} className="waypoint-item">
            <span>{index + 1}: {missionItemTypes[waypoint.type].name}</span>
            <span>Lat: {waypoint.lat.toFixed(6)}</span>
            <span>Lon: {waypoint.lng.toFixed(6)}</span>
            <span>Alt: {waypoint.altitude}m</span>
            {Object.entries(waypoint.params).map(([param, value]) => (
              <span key={param}>{param}: {value}</span>
            ))}
            <Button
              onClick={() => {
                const newWaypoints = waypoints.filter((_, i) => i !== index);
                setWaypoints(newWaypoints);
              }}
              className="remove-waypoint-button"
            >
              Remove
            </Button>
          </div>
        ))}
      </div>
      
      <div className="status-display">
        <h2>Drone Status</h2>
        <pre>
          {JSON.stringify(droneStatus, null, 2)}
        </pre>
      </div>
    </div>
  );
};

// Helper function to calculate distance between two points
function calculateDistance(point1, point2) {
  const R = 6371e3; // Earth's radius in meters
  const φ1 = point1.lat * Math.PI / 180;
  const φ2 = point2.lat * Math.PI / 180;
  const Δφ = (point2.lat - point1.lat) * Math.PI / 180;
  const Δλ = (point2.lng - point1.lng) * Math.PI / 180;

  const a = Math.sin(Δφ/2) * Math.sin(Δφ/2) +
            Math.cos(φ1) * Math.cos(φ2) *
            Math.sin(Δλ/2) * Math.sin(Δλ/2);
  const c = 2 * Math.atan2(Math.sqrt(a), Math.sqrt(1-a));

  return R * c;
}

export default DroneControlPanel;