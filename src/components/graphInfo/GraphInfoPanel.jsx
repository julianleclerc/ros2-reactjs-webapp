import React from 'react';
import { Light as SyntaxHighlighter } from 'react-syntax-highlighter';
import json from 'react-syntax-highlighter/dist/esm/languages/hljs/json';
import { vs2015 } from 'react-syntax-highlighter/dist/esm/styles/hljs';
import './GraphInfoPanel.css';

// Register the JSON language
SyntaxHighlighter.registerLanguage('json', json);

const GraphInfoPanel = ({ graphDataIn, nodeDataIn, actionDataIn }) => {
  if (!graphDataIn && !nodeDataIn && !actionDataIn) {
    return (
      <div className="graph-info-container">
        <h2>Details</h2>
        <pre className="placeholder-text">Select a graph, node, or action</pre>
      </div>
    );
  }

  // Determine which data to show and what title to display
  const getDisplayData = () => {
    if (nodeDataIn) return { title: 'Node Details', data: nodeDataIn };
    if (actionDataIn) return { title: 'Action Details', data: actionDataIn };
    return { title: 'Graph Details', data: graphDataIn };
  };

  const { title, data } = getDisplayData();

  return (
    <div className="graph-info-container">
      <h2>{title}</h2>
      <SyntaxHighlighter 
        language="json" 
        style={vs2015}
        customStyle={{
          borderRadius: '5px',
          margin: '0',
          padding: '15px',
          width: '100%',
          boxShadow: '0 2px 5px rgba(0, 0, 0, 0.2)',
        }}
      >
        {JSON.stringify(data, null, 2)}
      </SyntaxHighlighter>
    </div>
  );
};

export default GraphInfoPanel;