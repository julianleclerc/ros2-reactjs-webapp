import React from "react";
import "./MenuPanel.css";

const MenuPanel = ({ setPage }) => {
    // Define button configurations
    const buttons = [
        { id: 1, text: "Chat", page: "ChatInterfacePage" },
        { id: 2, text: "Action", page: "ActionInterfacePage" },
    ];

    // Render the menu panel with buttons
    return (
        <div className="menu-panel">
            {buttons.map((button) => (
                <button
                    key={button.id}
                    className="menu-button"
                    onClick={() => setPage(button.page)}
                >
                    {button.text} 
                </button>
            ))}
        </div>
    );
};

export default MenuPanel;
