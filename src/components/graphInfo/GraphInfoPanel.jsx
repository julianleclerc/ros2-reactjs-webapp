import React, { useState, useEffect } from 'react';
import { Light as SyntaxHighlighter } from 'react-syntax-highlighter';
import json from 'react-syntax-highlighter/dist/esm/languages/hljs/json';
import { vs2015 } from 'react-syntax-highlighter/dist/esm/styles/hljs';
import './GraphInfoPanel.css';

// Register the JSON language
SyntaxHighlighter.registerLanguage('json', json);

const GraphInfoPanel = ({ selectedElement, isNewAction, onActionGenerated }) => {
  const [actionData, setActionData] = useState({
    name: '',
    description: '',
    parameters: {}
  });
  const [generationSuccess, setGenerationSuccess] = useState(false);

  useEffect(() => {
    if (selectedElement.type === 'action' && selectedElement.data) {
      setActionData(selectedElement.data);
      setGenerationSuccess(false); // Reset success state when new action is selected
    }
  }, [selectedElement]);

  const handleGenerateAction = async () => {
    try {
      const response = await fetch('http://localhost:4000/api/generate-action', {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json'
        },
        body: JSON.stringify(actionData)
      });

      if (!response.ok) {
        throw new Error('Failed to generate action');
      }

      const result = await response.json();
      console.log('Action generated successfully:', result);
      setGenerationSuccess(true);
      if (onActionGenerated) {
        onActionGenerated(); // Pass the generated action back
      }
    } catch (error) {
      console.error('Error generating action:', error);
    }
  };

  if (!selectedElement.data) {
    return (
      <div className="graph-info-container">
        <h2>Details</h2>
        <pre className="placeholder-text">Select a graph, node, or action</pre>
      </div>
    );
  }

  // Determine title based on the type
  const getTitle = () => {
    switch (selectedElement.type) {
      case 'graph': return 'Graph Details';
      case 'node': return 'Node Details';
      case 'action': return 'Action Details';
      default: return 'Details';
    }
  };

  return (
    <div className="graph-info-container">
      <h2>{getTitle()}</h2>
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
        {JSON.stringify(selectedElement.data, null, 2)}
      </SyntaxHighlighter>
      {isNewAction && selectedElement.type === 'action' && !generationSuccess && (
        <button 
          className="generate-button"
          onClick={handleGenerateAction}
        >
          Generate Action
        </button>
      )}
      {generationSuccess && (
        <div className="success-message">
          Action Generated Successfully!
        </div>
      )}
    </div>
  );
};

export default GraphInfoPanel;