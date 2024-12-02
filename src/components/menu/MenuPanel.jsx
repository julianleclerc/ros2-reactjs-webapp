import React from "react";
import "./MenuPanel.css";

const MenuPanel = ({ setPage }) => {
    // Define button configurations for the menu
    const buttons = [
        { id: 1, text: "1", tooltip: "Chat interface", page: "ChatInterfacePage" },
        { id: 2, text: "2", tooltip: "Action edit interface", page: "ActionInterfacePage" },
    ];

    // Render the menu panel with buttons
    return (
        <div className="menu-panel">
            {buttons.map((button) => (
                <button
                    key={button.id} // Unique key for each button
                    className="menu-button" // Shared button styling
                    data-tooltip={button.tooltip} // Tooltip text
                    onClick={() => setPage(button.page)} // Set the page on click
                >
                    {button.text} {/* Display button text */}
                </button>
            ))}
        </div>
    );
};

export default MenuPanel;
