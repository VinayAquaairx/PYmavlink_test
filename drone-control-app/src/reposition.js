import React, { useState, useEffect } from 'react';
import { MapContainer, TileLayer, useMapEvents, Marker, Popup } from 'react-leaflet';
import 'leaflet/dist/leaflet.css';
import axios from 'axios';

const DroneMap = () => {
  const [position, setPosition] = useState(null);
  const [altitude, setAltitude] = useState('');
  const [homePosition, setHomePosition] = useState(null);

  const MapClickHandler = () => {
    useMapEvents({
      click: (e) => {
        setPosition([e.latlng.lat, e.latlng.lng]);
      },
    });
    return null;
  };

  const handleAltitudeChange = (e) => {
    setAltitude(e.target.value);
  };

  const handleSendPosition = async () => {
    if (position && altitude) {
      const [latitude, longitude] = position;
      try {
        const response = await axios.post('http://localhost:5001/set_position', {
          latitude,
          longitude,
          altitude: parseFloat(altitude),
        });
        alert('Drone repositioned successfully');
      } catch (error) {
        console.error('Error setting position:', error.response?.data || error.message);
        alert('Failed to reposition the drone.');
      }
    } else {
      alert('Please select a position and provide an altitude.');
    }
  };

  useEffect(() => {
    const fetchHomePosition = async () => {
      try {
        const response = await axios.get('http://localhost:5001/get_home_position');
        const { latitude, longitude } = response.data;
        setHomePosition([latitude, longitude]);
      } catch (error) {
        console.error('Error fetching home position:', error.response?.data || error.message);
      }
    };

    fetchHomePosition();
  }, []);

  return (
    <div className="container mx-auto p-4">
      <h1 className="text-2xl font-bold mb-4">Drone Repositioning</h1>
      <MapContainer center={[0, 0]} zoom={3} style={{ height: '400px', width: '100%' }}>
        <TileLayer url="https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png" />
        <MapClickHandler />
        {homePosition && (
          <Marker position={homePosition}>
            <Popup>Home Position</Popup>
          </Marker>
        )}
        {position && (
          <Marker position={position}>
            <Popup>
              <div>
                <p>New Position</p>
                <input type="number" value={altitude} onChange={handleAltitudeChange} placeholder="Enter altitude (m)" className="mb-2 p-1 border rounded"/>
                <button onClick={handleSendPosition}className="bg-blue-500 text-white px-2 py-1 rounded"> Go to this position</button>
              </div>
            </Popup>
          </Marker>
        )}
      </MapContainer>
    </div>
  );
};

export default DroneMap;