import React, { useState, useEffect, useRef } from 'react';
import axios from 'axios';

const ParameterList = () => {
  const [parameters, setParameters] = useState([]);
  const [isLoadingParameters, setIsLoadingParameters] = useState(false);
  const [alertMessage, setAlertMessage] = useState('');
  const [showParameterList, setShowParameterList] = useState(false);
  const listRef = useRef(null);
  const [scrollPosition, setScrollPosition] = useState(0);

  // Set the base URL for your backend
  const API_BASE_URL = 'http://localhost:5001';

  const fetchParameters = async () => {
    setIsLoadingParameters(true);
    try {
      // First, trigger parameter fetching
      await axios.get(`${API_BASE_URL}/fetch_parameters`);
      
      // Then start polling for results
      pollParameters();
    } catch (error) {
      console.error('Error fetching parameters:', error);
      setAlertMessage('Failed to fetch parameters');
      setIsLoadingParameters(false);
    }
  };

  const pollParameters = async () => {
    const interval = setInterval(async () => {
      try {
        const response = await axios.get(`${API_BASE_URL}/get_parameters`);
        if (response.data.status === 'complete') {
          // Convert the parameters array to the format your component expects
          const formattedParameters = response.data.parameters.map(([key, value]) => ({
            name: key,
            value: value.value,
            type: value.type
          }));
          setParameters(formattedParameters);
          clearInterval(interval);
          setIsLoadingParameters(false);
        } else if (response.data.status === 'fetching') {
          // Still fetching, continue polling
          setParameters(response.data.parameters);
        }
      } catch (error) {
        console.error('Error polling parameters:', error);
        clearInterval(interval);
        setIsLoadingParameters(false);
        setAlertMessage('Error while fetching parameters');
      }
    }, 1000);
  };

  const handleCheckFullParams = () => {
    setShowParameterList(true);
    fetchParameters();
  };

  useEffect(() => {
    if (listRef.current) {
      listRef.current.scrollTop = scrollPosition;
    }
  }, [parameters, scrollPosition]);

  const handleScroll = () => {
    if (listRef.current) {
      setScrollPosition(listRef.current.scrollTop);
    }
  };

  const handleParameterChange = async (paramName, newValue) => {
    try {
      const response = await axios.post(`${API_BASE_URL}/set_parameter`, {
        param_id: paramName,
        param_value: newValue
      });
      if (response.data.status === 'success') {
        // Update the parameter in the local state
        setParameters(prevParams => 
          prevParams.map(param => 
            param.name === paramName ? { ...param, value: response.data.new_value } : param
          )
        );
        setAlertMessage(`Parameter ${paramName} updated successfully`);
      }
    } catch (error) {
      console.error('Error updating parameter:', error);
      setAlertMessage(`Failed to update parameter ${paramName}`);
    }
  };

  return (
    <div>
      <button onClick={handleCheckFullParams}>Check Full Parameters</button>
      {showParameterList && (
        <div ref={listRef} onScroll={handleScroll} style={{ maxHeight: '400px', overflowY: 'auto' }}>
          <h2>Parameter List</h2>
          {isLoadingParameters ? (
            <p>Loading parameters...</p>
          ) : (
            <table>
              <thead>
                <tr>
                  <th>Name</th>
                  <th>Value</th>
                  <th>Action</th>
                </tr>
              </thead>
              <tbody>
                {parameters.map((param) => (
                  <tr key={param.name}>
                    <td>{param.name}</td>
                    <td>{param.value}</td>
                    <td>
                      <input
                        type="number"
                        defaultValue={param.value}
                        onBlur={(e) => handleParameterChange(param.name, e.target.value)}
                      />
                    </td>
                  </tr>
                ))}
              </tbody>
            </table>
          )}
        </div>
      )}
      {alertMessage && <p>{alertMessage}</p>}
    </div>
  );
};

export default ParameterList;