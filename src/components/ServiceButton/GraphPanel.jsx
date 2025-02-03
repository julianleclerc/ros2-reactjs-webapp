import React, { useState } from 'react';

function GraphPanel() {
  const [indexedGraphs, setIndexedGraphs] = useState([]);
  const [runningGraphs, setRunningGraphs] = useState([]);
  const [message, setMessage] = useState('');

  // Function to fetch and display the graphs
  const getUmrfGraphsService = async () => {
    try {
      const response = await fetch('http://localhost:4000/get-umrf-graphs-ros-service', {
        method: 'POST',
      });

      const data = await response.json();
      
      if (response.ok) {
        // Set the graph data
        setIndexedGraphs(data.graph_jsons_indexed || []);
        setRunningGraphs(data.graph_jsons_running || []);
        setMessage('Graphs loaded successfully!');
      } else {
        setMessage('Error: ' + data.message);
      }
    } catch (error) {
      setMessage('Error: ' + error.message); // Handle network errors
    }
  };

  return (
    <div>
      <button onClick={getUmrfGraphsService}>Load Graphs</button>

      {message && <p>{message}</p>}

      <div className="panel">
        <h2>Indexed Graphs</h2>
        {indexedGraphs.length > 0 ? (
          <ul>
            {indexedGraphs.map((graph, index) => (
              <li key={index}>{JSON.stringify(graph)}</li>
            ))}
          </ul>
        ) : (
          <p>No indexed graphs found.</p>
        )}

        <h2>Running Graphs</h2>
        {runningGraphs.length > 0 ? (
          <ul>
            {runningGraphs.map((graph, index) => (
              <li key={index}>{JSON.stringify(graph)}</li>
            ))}
          </ul>
        ) : (
          <p>No running graphs found.</p>
        )}
      </div>
    </div>
  );
}

export default GraphPanel;