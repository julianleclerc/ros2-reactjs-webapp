import React, { useState, useEffect } from 'react';
import './GraphListPanel.css'; // Import the CSS file

const GraphListPanel = ({ graphsIn, onGraphSelect, onStartStopClick }) => {
    const [graphs, setGraphs] = useState([]);
    const [activeGraph, setActiveGraph] = useState();

    useEffect(() => {
        if (graphsIn) {
            setGraphs(graphsIn)
        }
    }, [graphsIn]);

    const handleGraphSelectClick = (graph) => {
        setActiveGraph(graph);
        onGraphSelect(graph.graph_name);
    };

    const handleStartStopClick = (graph) => {
        onStartStopClick(graph.graph_name);
    };

    return (
        <div>
            <div className="buttons-column">
                <h2>Graphs</h2>
                {graphs.map((graph, index) => (
                    <div className="buttons-row">

                        {/* Button for selecting the graph */}
                        <button
                            className={`text-button ${activeGraph?.graph_name === graph.graph_name ? 'active' : ''}`}
                            key={index}
                            onClick={() => handleGraphSelectClick(graph)}
                        >
                            {graph.graph_name}
                        </button>

                        {/* Button for starting/stopping the graph */}
                        <button
                            className={`icon-button ${graph.graph_state === "RUNNING" ? 'running' : ''}`}
                            onClick={() => handleStartStopClick(graph)}
                        >
                            <svg
                                xmlns="http://www.w3.org/2000/svg"
                                width="16"
                                height="16"
                                fill="currentColor"
                                className="bi bi-play-fill"
                                viewBox="0 0 16 16"
                            >
                                {graph.graph_state === "RUNNING" ? (<rect x="4" y="4" width="8" height="8" />) : (<path d="M3 2 L13 8 L3 14 Z"/>)}
                            </svg>
                        </button>

                    </div>
                ))}

            </div>
        </div>
    );
};

export default GraphListPanel;
