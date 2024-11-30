import React from "react";
import "./MenuPanel.css";

const MenuPanel = ({ setPage }) => {
    const buttons = [
        { id: 1, text: "1", tooltip: "Chat interface", page: "ChatInterfacePage" },
        { id: 2, text: "2", tooltip: "Action edit interface", page: "ActionInterfacePage" },
    ];

    return (
        <div className="menu-panel">
            {buttons.map((button) => (
                <button
                    key={button.id}
                    className="menu-button"
                    data-tooltip={button.tooltip} // Tooltip text
                    onClick={() => setPage(button.page)} // Set the selected page
                >
                    {button.text}
                </button>
            ))}
        </div>
    );
};

export default MenuPanel;
