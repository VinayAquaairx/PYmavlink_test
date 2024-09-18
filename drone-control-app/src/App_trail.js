import React, { useState, useEffect } from 'react';
import axios from 'axios';
import './DroneControlPanel.css';

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

const DroneControlPanel = () => {
  const [droneStatus, setDroneStatus] = useState({});
  const [isArmed, setIsArmed] = useState(false);
  const [showAltitudeModal, setShowAltitudeModal] = useState(false);
  const [altitude, setAltitude] = useState('');
  const [alertMessage, setAlertMessage] = useState('');
  const [modeName, setModeName] = useState('');

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

  useEffect(() => {
    const fetchStatus = async () => {
      try {
        const response = await axios.get('http://localhost:5001/status');
        setDroneStatus(response.data);
        setIsArmed(response.data.armed === 128);
        setModeName(modeMap[response.data.mav_mode] || 'Unknown');
      } catch (error) {
        console.error('Error fetching drone status:', error);
      }
    };

    fetchStatus();
    const intervalId = setInterval(fetchStatus, 1000);

    return () => clearInterval(intervalId);
  }, []);

  const handleCommand = async (command, params = {}) => {
    try {
      const response = await axios.post('http://localhost:5001/command', { command, ...params });
      if (response.data.status === 'success') {
        setAlertMessage(`${command} command executed successfully`);
      } else {
        setAlertMessage(`Failed to execute ${command} command`);
      }
    } catch (error) {
      console.error(`Error executing ${command} command:`, error);
      setAlertMessage(`Error executing ${command} command`);
    }
  };

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

      <div className="status-display">
        <h2>Drone Status</h2>
        <pre>
          {JSON.stringify(droneStatus, null, 2)}
        </pre>
      </div>
    </div>
  );
};

export default DroneControlPanel;