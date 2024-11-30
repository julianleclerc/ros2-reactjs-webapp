import React from 'react';
import './ChatInterfacePage.css';
import ChatPanel from '../../components/chat/ChatPanel.jsx';
import MenuPanel from '../../components/menu/MenuPanel.jsx';
import PlannedActionPanel from '../../components/plannedActions/PlannedActionPanel.jsx';
import DisplayPanel from '../../components/display/DisplayPanel.jsx';

const ChatInterfacePage = () => {
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
};

export default ChatInterfacePage;
