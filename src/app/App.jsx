import React from 'react';
import './App.css';
import ChatPanel from '../features/chat/ChatPanel';
import MenuPanel from '../features/menu/MenuPanel';
import PlannedActionPanel from '../features/plannedActions/PlannedActionPanel';
import DisplayPanel from '../features/display/DisplayPanel';

const App = () => {
    return (
        <div className="app">
            <div className="menu-panel">
                <MenuPanel />
            </div>
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

export default App;
