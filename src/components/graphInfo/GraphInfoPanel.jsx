import React, { useState, useEffect } from 'react';
import { Light as SyntaxHighlighter } from 'react-syntax-highlighter';
import json from 'react-syntax-highlighter/dist/esm/languages/hljs/json';
import { vs2015 } from 'react-syntax-highlighter/dist/esm/styles/hljs';
import './GraphInfoPanel.css';

// Register the JSON language
SyntaxHighlighter.registerLanguage('json', json);

const GraphInfoPanel = ({ selectedElement, onActionGenerated, onActionUpdated }) => {
  const [actionData, setActionData] = useState({
    name: '',
    description: '',
    parameters: {},
    status: ''
  });
  const [generationSuccess, setGenerationSuccess] = useState(false);
  const [isEditingName, setIsEditingName] = useState(false);
  const [editedName, setEditedName] = useState('');

  useEffect(() => {
    if (selectedElement.type === 'action' && selectedElement.data) {
      setActionData(selectedElement.data);
      setEditedName(selectedElement.data.name);
      setGenerationSuccess(false); // Reset success state when new action is selected
      setIsEditingName(false); // Reset editing state when switching actions
    }
  }, [selectedElement]);

  const handleGenerateAction = async () => {
    try {
      // Make sure we're using the most up-to-date action data with edited name
      const actionToGenerate = {
        ...actionData,
        name: editedName
      };

      const response = await fetch('http://localhost:4000/api/generate-action', {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json'
        },
        body: JSON.stringify(actionToGenerate)
      });

      if (!response.ok) {
        throw new Error('Failed to generate action');
      }

      const result = await response.json();
      console.log('Action generated successfully:', result);
      setGenerationSuccess(true);
      
      // Add status to the result if it doesn't already have one
      const generatedAction = {
        ...result,
        status: 'package'
      };
      
      if (onActionGenerated) {
        onActionGenerated(generatedAction);
      }
    } catch (error) {
      console.error('Error generating action:', error);
    }
  };

  const handleNameEdit = () => {
    if (selectedElement.type === 'action' && selectedElement.data.status === 'draft') {
      setIsEditingName(true);
    }
  };

  const handleNameSave = () => {
    if (editedName.trim() === '') {
      // Don't allow empty names
      setEditedName(actionData.name);
      setIsEditingName(false);
      return;
    }

    // Update the action data with the new name
    const updatedAction = {
      ...actionData,
      name: editedName
    };

    setActionData(updatedAction);
    setIsEditingName(false);

    // Notify parent component about the name change
    if (onActionUpdated) {
      onActionUpdated(updatedAction);
    }
  };

  const handleNameChange = (e) => {
    setEditedName(e.target.value);
  };

  const handleKeyDown = (e) => {
    if (e.key === 'Enter') {
      handleNameSave();
    } else if (e.key === 'Escape') {
      setEditedName(actionData.name);
      setIsEditingName(false);
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

  // Create a modified version of the data for display
  const getDisplayData = () => {
    if (selectedElement.type === 'action') {
      // Create a copy to avoid modifying the original
      const displayData = { ...selectedElement.data };
      
      // If we're editing the name, show the edited name in the JSON display
      if (isEditingName) {
        displayData.name = editedName;
      }
      
      return displayData;
    }
    
    return selectedElement.data;
  };

  return (
    <div className="graph-info-container">
      <div className="info-header">
        <h2>{getTitle()}</h2>
        
        {/* Show status badge if it's an action */}
        {selectedElement.type === 'action' && (
          <div className={`status-badge ${selectedElement.data.status || 'unknown'}`}>
            {selectedElement.data.status === 'draft' ? 'Draft' : 
             selectedElement.data.status === 'package' ? 'Package' : 'Package'}
          </div>
        )}
        
        {/* Add name editing for draft actions */}
        {selectedElement.type === 'action' && selectedElement.data.status === 'draft' && (
          <div className="action-name-editor">
            {isEditingName ? (
              <div className="name-input-container">
                <input
                  type="text"
                  value={editedName}
                  onChange={handleNameChange}
                  onBlur={handleNameSave}
                  onKeyDown={handleKeyDown}
                  autoFocus
                  className="name-input"
                />
                <button className="save-name-button" onClick={handleNameSave}>
                  Save
                </button>
              </div>
            ) : (
              <div className="name-display" onClick={handleNameEdit}>
                <span className="action-name">{actionData.name}</span>
                <span className="edit-icon">✏️</span>
              </div>
            )}
          </div>
        )}
      </div>
      
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
        {JSON.stringify(getDisplayData(), null, 2)}
      </SyntaxHighlighter>
      
      {/* Only show Generate button for draft actions */}
      {selectedElement.type === 'action' && 
       selectedElement.data.status === 'draft' && 
       !generationSuccess && (
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