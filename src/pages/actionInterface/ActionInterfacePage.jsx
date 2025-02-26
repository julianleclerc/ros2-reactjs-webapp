import React, { useState, useEffect, useRef } from 'react';
import io from 'socket.io-client';
import GraphListPanel from "../../components/graphList/GraphListPanel.jsx";
import NodeEditorPanel from "../../components/nodeEditor/NodeEditorPanel.jsx";
import GraphInfoPanel from "../../components/graphInfo/GraphInfoPanel.jsx";
import "./ActionInterfacePage.css";

const ActionInterfacePage = () => {

    const [graphs, setGraphs] = useState(null);
    const [selectedGraph, setSelectedGraph] = useState(null);
    const [selectedAction, setSelectedAction] = useState(null);
    const [runtimeEnabled, setRuntimeEnabled] = useState(false);
    const nodeEditorRef = useRef();

    const handleGraphSelect = async (graphName) => {
        console.log('clicked graph!', graphName);
        
        // Always reset selected action regardless of which graph is clicked
        setSelectedAction(null);
        
        // Only fetch the graph data if it's a different graph
        if (selectedGraph && selectedGraph.graph_name === graphName) {
            return;
        }
        
        // Make the NodeEditorPanel spit out currently active graph,
        // which gets saved in the handleGetCurrentGraph
        if (nodeEditorRef.current) {
            nodeEditorRef.current.getCurrentGraph();
        }

        // Fetch the new graph
        try {
            const response = await fetch(`http://localhost:4000/api/graphs/${graphName}`);
            if (!response.ok) {
                throw new Error('Network response was not ok');
            }

            const result = await response.json();
            setSelectedGraph(result);

        } catch (error) {
            console.error('Error fetching data', error);
        }
    };

    const handleGetCurrentGraph = async (updatedGraph) => {
        try {
            const response = await fetch(`http://localhost:4000/api/graphs/${updatedGraph.graph_name}`, {
                method: 'PUT',
                headers: {
                    'Content-Type': 'application/json'
                },
                body: JSON.stringify(updatedGraph)
            });

            if (!response.ok) {
                throw new Error('Network response was not ok');
            }

            const result = await response.json();
            console.log(result.message);
        } catch (error) {
            console.error('Error updating graph:', error);
        }
    };

    const handleStartStopClick = async (graphName) => {
        try {
            const response = await fetch(`http://localhost:4000/api/graphs/exec/${graphName}`, {
                method: 'PUT',
                headers: {
                    'Content-Type': 'application/json'
                }
            });

            if (!response.ok) {
                throw new Error('Network response was not ok');
            }

            const result = await response.json();
            console.log(result.message);

        } catch (error) {
            console.error('Error updating graph:', error);
        }
    };

    const handleActionSelect = (action) => {
        setSelectedAction(action);
    };

    useEffect(() => {
        const socket = io('http://localhost:4000');

        socket.on('graphs', (data) => {
            setGraphs(data);
        });

        socket.on('runtime_enabled', (data) => {
            setRuntimeEnabled(data);
        });

        return () => {
            socket.disconnect();
        };
    }, []);

    // Update the rendered graph when the state changes
    useEffect(() => {
        if (graphs && selectedGraph) {
            setSelectedGraph(graphs.find(graph => graph.graph_name === selectedGraph.graph_name));
        }
    }, [graphs]);

    return (
        <div className="action-interface-page">
            <div className="graph-list-panel">
                <GraphListPanel
                    graphsIn={graphs}
                    onGraphSelect={handleGraphSelect}
                    onStartStopClick={handleStartStopClick}
                    runtimeEnabled={runtimeEnabled}/>
            </div>

            <div className="node-editor-panel">
                <NodeEditorPanel
                    ref={nodeEditorRef}
                    graphDataIn={selectedGraph}
                    onUpdateGraph={handleGetCurrentGraph}
                    onActionSelect={handleActionSelect}/>
            </div>
            <div className="graph-info-panel">
                <GraphInfoPanel
                    graphDataIn={selectedGraph}
                    actionDataIn={selectedAction}
                />
            </div>
        </div>
    );
};

export default ActionInterfacePage;
