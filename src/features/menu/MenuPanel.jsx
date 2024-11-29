import React, { useState } from "react";
import "./MenuPanel.css";

const MenuPanel = () => {
    const [selected, setSelected] = useState(1); // Default selected button is 1

    const buttons = [
        { id: 1, text: "1", tooltip: "Chat interface" },
        { id: 2, text: "2", tooltip: "Action edit interface" },
    ];

    return (
        <div className="menu-panel">
            {buttons.map((button) => (
                <button
                    key={button.id}
                    className={selected === button.id ? "active" : ""}
                    data-tooltip={button.tooltip} // Tooltip text
                    onClick={() => setSelected(button.id)} // Set the selected button
                >
                    {button.text}
                </button>
            ))}
        </div>
    );
};

export default MenuPanel;
