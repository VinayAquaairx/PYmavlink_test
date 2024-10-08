import React from 'react';
import ReactDOM from 'react-dom/client';
import './index.css';
import App from './App';
import App_trail from './App_trail' 
import reportWebVitals from './reportWebVitals';
import Accecal from './Accecal'


const root = ReactDOM.createRoot(document.getElementById('root'));
root.render(
  <React.StrictMode>
    {/* <App_trail/> */}
    <Accecal/>
    </React.StrictMode>
);

// If you want to start measuring performance in your app, pass a function
// to log results (for example: reportWebVitals(console.log))
// or send to an analytics endpoint. Learn more: https://bit.ly/CRA-vitals
reportWebVitals();
