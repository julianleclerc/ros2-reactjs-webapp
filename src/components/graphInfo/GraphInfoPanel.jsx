import React from 'react';
import { Light as SyntaxHighlighter } from 'react-syntax-highlighter';
import json from 'react-syntax-highlighter/dist/esm/languages/hljs/json';
import { vs2015 } from 'react-syntax-highlighter/dist/esm/styles/hljs';
import './GraphInfoPanel.css';

// Register the JSON language
SyntaxHighlighter.registerLanguage('json', json);

const GraphInfoPanel = ({ graphDataIn, actionDataIn }) => {
  if (!graphDataIn) {
    return (
      <div className="graph-info-container">
        <h2>Graph Details</h2>
        <pre className="placeholder-text">Select a graph or action</pre>
      </div>
    );
  }

  return (
    <div className="graph-info-container">
      <h2>{actionDataIn === null ? 'Graph Details' : 'Action Details'}</h2>
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
        {JSON.stringify(actionDataIn === null ? graphDataIn : actionDataIn, null, 2)}
      </SyntaxHighlighter>
    </div>
  );
};

export default GraphInfoPanel;