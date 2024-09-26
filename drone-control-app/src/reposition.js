import React, { useState, useEffect } from 'react';
import { MapContainer, TileLayer, Marker, Popup, useMapEvents } from 'react-leaflet';
import L from 'leaflet';
import 'leaflet/dist/leaflet.css';
import axios from 'axios';

const DroneMap = () => {
  const [position, setPosition] = useState(null);
  const [homePosition, setHomePosition] = useState(null);
  const [menuPosition, setMenuPosition] = useState(null); // State to handle context menu position
  const [showMenu, setShowMenu] = useState(false); // State to show/hide the context menu

  const MapEvents = () => {
    const map = useMapEvents({
      contextmenu: (e) => {
        // Handle right-click to open context menu
        setMenuPosition(e.latlng);
        setShowMenu(true);
        e.originalEvent.preventDefault(); // Prevent the default context menu
      }
    });
    return null;
  };

  const handleSendPosition = async (altitude) => {
    if (menuPosition && altitude) {
      try {
        const response = await axios.post('http://localhost:5001/set_position', {
          latitude: menuPosition.lat,
          longitude: menuPosition.lng,
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
    <div>
      <h3>Drone Repositioning</h3>
      <MapContainer center={[51.505, -0.09]} zoom={13} style={{ height: '400px', width: '100%' }}>
        <TileLayer url="https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png" />
        {homePosition && <Marker position={homePosition} />}
        <MapEvents />
      </MapContainer>
      {showMenu && (
        <div style={{ position: 'absolute', top: menuPosition.y, left: menuPosition.x }}>
          <div style={{ background: 'white', padding: '10px', borderRadius: '5px', boxShadow: '0 0 15px rgba(0,0,0,0.2)' }}>
            <ul style={{ listStyle: 'none', margin: 0, padding: 0 }}>
              <li>
                <button onClick={() => {
                  const altitude = prompt('Enter altitude:');
                  if (altitude) handleSendPosition(altitude);
                  setShowMenu(false);
                }}>Go to this position</button>
              </li>
            </ul>
          </div>
        </div>
      )}
    </div>
  );
};

export default DroneMap;
