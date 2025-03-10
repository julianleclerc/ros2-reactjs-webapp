import React, { useState, useEffect, forwardRef, useImperativeHandle } from 'react';
import './GraphListPanel.css'; // Import the CSS file

const GraphListPanel = forwardRef(({ graphsIn, onGraphSelect, onStartStopClick, onNewGraph, runtimeEnabled }, ref) => {
    const [graphs, setGraphs] = useState([]);
    const [activeGraph, setActiveGraph] = useState(null);
    
    useEffect(() => {
        if (graphsIn) {
            setGraphs(graphsIn)
        }
    }, [graphsIn]);

    useImperativeHandle(ref, () => ({
        clearActiveGraph: () => {
            setActiveGraph(null);
        },
        setActiveGraph: (graph) => {
            setActiveGraph(graph);
        }
    }));

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
                    <div className="buttons-row" key={index}>
                        <button
                            className={`text-button ${activeGraph?.graph_name === graph.graph_name ? 'active' : ''}`}
                            onClick={() => handleGraphSelectClick(graph)}
                        >
                            {graph.graph_name}
                        </button>

                        {runtimeEnabled && (
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
                                    {graph.graph_state === "RUNNING" ? 
                                        (<rect x="4" y="4" width="8" height="8" />) : 
                                        (<path d="M3 2 L13 8 L3 14 Z"/>)
                                    }
                                </svg>
                            </button>
                        )}
                    </div>
                ))}
                
                <div className="buttons-row">
                    <button
                        className="text-button"
                        onClick={onNewGraph}
                    >
                        + 
                    </button>
                </div>
            </div>
        </div>
    );
});

export default GraphListPanel;
