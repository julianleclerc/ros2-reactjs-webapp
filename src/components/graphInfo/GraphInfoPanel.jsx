import React from 'react';
import './GraphInfoPanel.css';

const GraphInfoPanel = ({ graphDataIn, actionDataIn }) => {
  if (!graphDataIn) {
    return <div>No graph selected</div>;
  }

  console.log("Graph Data In: ", graphDataIn);
  console.log("Action Data In: ", actionDataIn);

  return (
    <div className="graph-info-container">
      {!actionDataIn ? (
        <pre>{JSON.stringify(graphDataIn, null, 2)}</pre>
      ) : (
        <pre>{JSON.stringify(actionDataIn, null, 2)}</pre>
      )}
    </div>
  );
};

export default GraphInfoPanel;