// src/pages/chatInterface/ChatInterfacePage.jsx

import React, { useState } from 'react';
import './ChatInterfacePage.css';
import ChatPanel from '../../components/chat/ChatPanel.jsx';
import DisplayPanel from '../../components/display/DisplayPanel.jsx';
import PlannedActionPanel from '../../components/plannedActions/PlannedActionPanel.jsx';
import useWindowDimensions from '../../hooks/useWindowDimensions';

const ChatInterfacePage = () => {
  const { aspectRatio } = useWindowDimensions();
  const [showPlannedActionPanel, setShowPlannedActionPanel] = useState(false);

  // Define the threshold aspect ratio
  const aspectRatioThreshold = 0.7;

  if (aspectRatio > aspectRatioThreshold) {
    // Mobile layout
    return (
      <div className="chat-interface-page mobile">
        {/* Button Container */}
        <div className="button-container">
          <button
            className="toggle-planned-action-panel-button"
            onClick={() => setShowPlannedActionPanel(!showPlannedActionPanel)}
          >
            {showPlannedActionPanel ? 'Hide Planned Actions' : 'Show Planned Actions'}
          </button>
        </div>

        {showPlannedActionPanel ? (
          <div className="planned-action-panel-container">
            <PlannedActionPanel />
          </div>
        ) : (
          <>
            <div className="display-panel">
              <DisplayPanel />
            </div>
            <div className="chat-panel">
              <ChatPanel />
            </div>
          </>
        )}
      </div>
    );
  } else {
    // Desktop layout
    return (
      <div className="chat-interface-page">
        <div className="planned-action-panel">
          <PlannedActionPanel />
        </div>
        <div className="display-panel">
          <DisplayPanel />
        </div>
        <div className="chat-panel">
          <ChatPanel />
        </div>
      </div>
    );
  }
};

export default ChatInterfacePage;
