import React, { useState, useEffect } from 'react';
import io from 'socket.io-client';
import GraphListPanel from "../../components/graphList/GraphListPanel.jsx";
import NodeEditorPanel from "../../components/nodeEditor/NodeEditorPanel.jsx";
import "./ActionInterfacePage.css";

const ActionInterfacePage = () => {

    const [graphNames, setGraphNames] = useState(null);
    const [selectedGraph, setSelectedGraph] = useState(null);

    const handleGraphSelect = async (graphName) => {
        console.log('clicked graph!', graphName);

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

    const handleUpdateGraph = async (updatedGraph) => {
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

    useEffect(() => {
        const socket = io('http://localhost:4000');

        socket.on('graphNames', (data) => {
            setGraphNames(data);
        });

        return () => {
            socket.disconnect();
        };
    }, []);

    return (
        <div className="action-interface-page">
            <div className="graph-list-panel">
                <GraphListPanel graphNamesIn={graphNames} onGraphSelect={handleGraphSelect}/>
            </div>

            <div className="node-editor-panel">
                <NodeEditorPanel graphDataIn={selectedGraph} onUpdateGraph={handleUpdateGraph}/>
            </div>
        </div>
    );
};

export default ActionInterfacePage;
