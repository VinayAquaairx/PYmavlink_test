import React, { useState, useEffect } from 'react';
import axios from 'axios';

function App() {
    const [status, setStatus] = useState("Idle");
    const [step, setStep] = useState(0);

    const steps = [
        { label: "Level Position", position: "level" },
        { label: "Left Side", position: "left" },
        { label: "Right Side", position: "right" },
        { label: "Nose Up", position: "nose_up" },
        { label: "Nose Down", position: "nose_down" },
        { label: "Tail Down", position: "tail_down" },
    ];

    useEffect(() => {
        const checkConnection = async () => {
            try {
                const response = await axios.get("http://localhost:5000/connect");
                setStatus(response.data.status);
            } catch (error) {
                setStatus("Connection failed");
            }
        };
        checkConnection();
    }, []);

    const startCalibration = async () => {
        try {
            setStatus("Starting calibration...");
            await axios.post("http://localhost:5000/calibrate/start");
            setStatus("Calibration started. Set to level position.");
        } catch (error) {
            setStatus(`Error: ${error.message}`);
        }
    };

    const setPosition = async () => {
        try {
            const currentStep = steps[step];
            setStatus(`Setting position: ${currentStep.label}...`);
            await axios.post("http://localhost:5000/calibrate/position", { position: currentStep.position });
            setStatus(`Position set: ${currentStep.label}.`);
            setStep(step + 1);
        } catch (error) {
            setStatus(`Error: ${error.message}`);
        }
    };

    const checkStatus = async () => {
        try {
            const response = await axios.get("http://localhost:5000/calibrate/status");
            setStatus(response.data.status);
        } catch (error) {
            setStatus(`Error: ${error.message}`);
        }
    };

    return (
        <div className="App">
            <h1>Accelerometer Calibration</h1>
            <p>Status: {status}</p>
            <button onClick={startCalibration} disabled={step > 0}>Start Calibration</button>
            <button onClick={setPosition} disabled={step >= steps.length || step === 0}>Set Position</button>
            <button onClick={checkStatus}>Check Status</button>
            <p>Current Step: {step < steps.length ? steps[step].label : "Calibration Complete"}</p>
        </div>
    );
}

export default App;
