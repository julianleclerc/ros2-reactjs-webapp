import React, { useState, useEffect } from 'react';
import './GraphListPanel.css'; // Import the CSS file

const GraphListPanel = ({ graphNamesIn, onGraphSelect }) => {
    const [graphNames, setGraphNames] = useState([]);
    const [activeGraph, setActiveGraph] = useState();

    useEffect(() => {
        if (graphNamesIn) {
            setGraphNames(graphNamesIn)
        }
    }, [graphNamesIn]);

    const handleButtonClick = (graphName) => {
        setActiveGraph(graphName);
        onGraphSelect(graphName);
    };

    return (
        <div>
            <div className="item-container">
                <h2>Graphs</h2>
                {graphNames.map((graphName, index) => (
                    <button
                        className={`item-button ${activeGraph === graphName ? 'active' : ''}`}
                        key={index}
                        onClick={() => handleButtonClick(graphName)}
                    >
                        {graphName}
                    </button>
                ))}

            </div>
        </div>
    );
};

export default GraphListPanel;
