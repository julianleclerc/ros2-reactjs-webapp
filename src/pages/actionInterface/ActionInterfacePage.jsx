import React, { useState, useEffect, useRef } from 'react';
import io from 'socket.io-client';
import GraphListPanel from "../../components/graphList/GraphListPanel.jsx";
import NodeEditorPanel from "../../components/nodeEditor/NodeEditorPanel.jsx";
import "./ActionInterfacePage.css";

const ActionInterfacePage = () => {

    const [graphs, setGraphs] = useState(null);
    const [selectedGraph, setSelectedGraph] = useState(null);
    const nodeEditorRef = useRef();

    const handleGraphSelect = async (graphName) => {
        console.log('clicked graph!', graphName);

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
            setSelectedGraph( result );

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

    useEffect(() => {
        const socket = io('http://localhost:4000');

        socket.on('graphs', (data) => {
            setGraphs(data);
        });

        return () => {
            socket.disconnect();
        };
    }, []);

    return (
        <div className="action-interface-page">
            <div className="graph-list-panel">
                <GraphListPanel graphsIn={graphs} onGraphSelect={handleGraphSelect} onStartStopClick={handleStartStopClick}/>
            </div>

            <div className="node-editor-panel">
                <NodeEditorPanel ref={nodeEditorRef} graphDataIn={selectedGraph} onUpdateGraph={handleGetCurrentGraph}/>
            </div>
        </div>
    );
};

export default ActionInterfacePage;
