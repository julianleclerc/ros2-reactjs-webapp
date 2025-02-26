import React from 'react';
import './GraphInfoPanel.css';

const GraphInfoPanel = ({ graphDataIn }) => {
  if (!graphDataIn) {
    return <div>No graph selected</div>;
  }

  return (
    <div className="graph-info-container">
      <h2>{graphDataIn.graph_name}</h2>
      <p>{graphDataIn.graph_description}</p>
      
      {/* Display the entire graphDataIn object as JSON */}
      <pre>{JSON.stringify(graphDataIn, null, 2)}</pre>
    </div>
  );
};

export default GraphInfoPanel;