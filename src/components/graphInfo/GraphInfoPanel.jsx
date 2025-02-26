import React from 'react';
import './GraphInfoPanel.css';

const GraphInfoPanel = ({ graphDataIn, actionDataIn }) => {
  if (!graphDataIn) {
    return <div className="graph-info-container">
      <h2>No Graph Selected</h2>
      <pre>Select a graph from the list to view details</pre>
    </div>;
  }

  return (
    <div className="graph-info-container">
      <h2>{actionDataIn === null ? 'Graph Details' : 'Action Details'}</h2>
      {actionDataIn === null ? (
        <pre>{JSON.stringify(graphDataIn, null, 2)}</pre>
      ) : (
        <pre>{JSON.stringify(actionDataIn, null, 2)}</pre>
      )}
    </div>
  );
};

export default GraphInfoPanel;