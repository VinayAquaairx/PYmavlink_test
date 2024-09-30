import React, { useState, useEffect } from 'react';
import { MapContainer, TileLayer, Marker, Popup, useMapEvents } from 'react-leaflet';
import L from 'leaflet';
import 'leaflet/dist/leaflet.css';

const icon = L.icon({
  iconUrl: 'https://cdnjs.cloudflare.com/ajax/libs/leaflet/1.7.1/images/marker-icon.png',
  iconSize: [25, 41],
  iconAnchor: [12, 41],
});

const buttonStyle = {
  padding: '10px 15px',
  margin: '0 5px',
  backgroundColor: '#4CAF50',
  color: 'white',
  border: 'none',
  borderRadius: '4px',
  cursor: 'pointer',
};

const inputStyle = {
  width: '100%',
  padding: '8px',
  margin: '5px 0',
  boxSizing: 'border-box',
};

const labelStyle = {
  fontWeight: 'bold',
  marginTop: '10px',
  display: 'block',
};

const selectStyle = {
  ...inputStyle,
  appearance: 'none',
  backgroundColor: 'white',
};

const DroneMissionPlanner = () => {
  const [waypoints, setWaypoints] = useState([]);
  const [showMissionInfo, setShowMissionInfo] = useState(false);
  const [error, setError] = useState(null);
  const addWaypoint = (latlng) => {
    setWaypoints([...waypoints, { ...latlng, altitude: 10, command: 16 }]);
  };

  const updateWaypoint = (index, updates) => {
    const newWaypoints = [...waypoints];
    newWaypoints[index] = { ...newWaypoints[index], ...updates };
    setWaypoints(newWaypoints);
  };

  const removeWaypoint = (index) => {
    setWaypoints(waypoints.filter((_, i) => i !== index));
  };

  const MapEvents = () => {
    useMapEvents({
      click(e) {
        addWaypoint(e.latlng);
      },
    });
    return null;
  };

  const writeMission = async () => {
    try {
      const response = await fetch('/mission', {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify({ waypoints }),
      });
      if (!response.ok) {
        const errorText = await response.text();
        throw new Error(`Failed to upload mission: ${response.status} ${response.statusText}\n${errorText}`);
      }
      const data = await response.json();
      alert('Mission uploaded successfully');
      console.log('Server response:', data);
    } catch (error) {
      console.error('Error uploading mission:', error);
      setError(`Error uploading mission: ${error.message}`);
    }
  };
  const readMission = async () => {
    try {
      const response = await fetch('/mission');
      if (!response.ok) {
        const errorText = await response.text();
        throw new Error(`Failed to read mission: ${response.status} ${response.statusText}\n${errorText}`);
      }
      const data = await response.json();
      setWaypoints(data.waypoints);
      console.log('Received waypoints:', data.waypoints);
    } catch (error) {
      console.error('Error reading mission:', error);
      setError(`Error reading mission: ${error.message}`);
    }
  };

  const startMission = async () => {
    try {
      const response = await fetch('/mission/start', { method: 'POST' });
      if (!response.ok) throw new Error('Failed to start mission');
      alert('Mission started successfully');
    } catch (error) {
      alert(`Error starting mission: ${error.message}`);
    }
  };

  const removeMission = async () => {
    try {
      const response = await fetch('/mission', { method: 'DELETE' });
      if (!response.ok) throw new Error('Failed to remove mission');
      setWaypoints([]);
      alert('Mission removed successfully');
    } catch (error) {
      alert(`Error removing mission: ${error.message}`);
    }
  };

  return (
    <div style={{ height: '100vh', display: 'flex', flexDirection: 'column' }}>
      <div style={{ display: 'flex', justifyContent: 'space-between', padding: '15px', backgroundColor: '#f0f0f0' }}>
        <button onClick={writeMission} style={buttonStyle}>Write Mission</button>
        <button onClick={readMission} style={buttonStyle}>Read Mission</button>
        <button onClick={startMission} style={buttonStyle}>Start Mission</button>
        <button onClick={removeMission} style={{...buttonStyle, backgroundColor: '#f44336'}}>Remove Mission</button>
      </div>{error && (
        <div style={{ padding: '10px', backgroundColor: '#ffcccb', color: '#d8000c', margin: '10px' }}>
          {error}
        </div>)}
      <MapContainer center={[51.505, -0.09]} zoom={13} style={{ height: '100%', width: '100%' }}>
        <TileLayer url="https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png" />
        <MapEvents />
        {waypoints.map((wp, index) => (
          <Marker key={index} position={[wp.lat, wp.lng]} icon={icon}>
            <Popup>
              <div style={{ padding: '10px' }}>
                <h3 style={{ marginBottom: '10px' }}>Waypoint {index + 1}</h3>
                <label style={labelStyle}>Altitude (m)</label>
                <input
                  type="number"
                  value={wp.altitude}
                  onChange={(e) => updateWaypoint(index, { altitude: parseFloat(e.target.value) })}
                  style={inputStyle}
                />
                <label style={labelStyle}>Command</label>
                <select
                  value={wp.command}
                  onChange={(e) => updateWaypoint(index, { command: parseInt(e.target.value) })}
                  style={selectStyle}
                >
                  <option value={16}>WAYPOINT</option>
                  <option value={20}>RETURN TO LAUNCH</option>
                  <option value={21}>LAND</option>
                </select>
                <button onClick={() => removeWaypoint(index)} style={{...buttonStyle, backgroundColor: '#f44336', marginTop: '10px', width: '100%'}}>Remove</button>
              </div>
            </Popup>
          </Marker>
        ))}
      </MapContainer>
      <button onClick={() => setShowMissionInfo(!showMissionInfo)} style={{...buttonStyle, margin: '15px'}}>
        {showMissionInfo ? 'Hide' : 'Show'} Mission Info
      </button>
      {showMissionInfo && (
        <div style={{ 
          position: 'absolute', 
          top: '50%', 
          left: '50%', 
          transform: 'translate(-50%, -50%)', 
          backgroundColor: 'white', 
          padding: '20px', 
          borderRadius: '5px', 
          boxShadow: '0 2px 10px rgba(0,0,0,0.1)',
          maxHeight: '80%',
          overflowY: 'auto'
        }}>
          <h2 style={{ marginBottom: '10px' }}>Mission Information</h2>
          <p>Total Waypoints: {waypoints.length}</p>
          {waypoints.map((wp, index) => (
            <div key={index} style={{ marginBottom: '5px' }}>
              Waypoint {index + 1}: Lat: {wp.lat.toFixed(6)}, Lng: {wp.lng.toFixed(6)}, Alt: {wp.altitude}m, Cmd: {wp.command}
            </div>
          ))}
          <button onClick={() => setShowMissionInfo(false)} style={{...buttonStyle, marginTop: '10px'}}>Close</button>
        </div>
      )}
    </div>
  );
};

export default DroneMissionPlanner;