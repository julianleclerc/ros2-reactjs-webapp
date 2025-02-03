import React, { useState } from 'react';

function CallGetUmrfGraphsButton() {
  const [message, setMessage] = useState('');

  // Function to handle the button click and make a POST request to Flask
  const getUmrfGraphsService = async () => {
    console.log('Button pressed: Initiating service call');
    
    try {
      const response = await fetch('http://localhost:4000/get-umrf-graphs-ros-service', {
        method: 'POST',
      });
  
      // Log the raw response to see if it's HTML or JSON
      const responseText = await response.text();
      console.log('Response Text:', responseText);
  
      // If the response is valid JSON, parse it
      const data = JSON.parse(responseText);
  
      if (response.ok) {
        console.log('Response received:', data);
        setMessage(data.message);
      } else {
        console.log('Error response received:', data);
        setMessage('Error: ' + data.message);
      }
    } catch (error) {
      console.error('Network or other error:', error);
      setMessage('Error: ' + error.message);
    }
  };
  

  return (
    <div>
      <button onClick={getUmrfGraphsService}>Call ROS Service</button>
      <p>{message}</p>
    </div>
  );
}

export default CallGetUmrfGraphsButton;