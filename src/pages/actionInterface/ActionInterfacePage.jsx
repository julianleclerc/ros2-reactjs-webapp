import React, { useState, useEffect, useRef } from 'react';
import io from 'socket.io-client';
import GraphListPanel from "../../components/graphList/GraphListPanel.jsx";
import ActionListPanel from "../../components/actionList/ActionListPanel.jsx";
import NodeEditorPanel from "../../components/nodeEditor/NodeEditorPanel.jsx";
import GraphInfoPanel from "../../components/graphInfo/GraphInfoPanel.jsx";
import "./ActionInterfacePage.css";

const ActionInterfacePage = () => {
    const [graphs, setGraphs] = useState(null);
    const [actions, setActions] = useState(null);
    const [selectedGraph, setSelectedGraph] = useState(null);
    const [selectedAction, setSelectedAction] = useState(null);
    const [selectedNode, setSelectedNode] = useState(null);
    const [runtimeEnabled, setRuntimeEnabled] = useState(false);
    const nodeEditorRef = useRef();
    const graphListRef = useRef();
    const actionListRef = useRef();

    console.log("graphs: ", graphs);
    console.log("actions: ", actions);

    const handleGraphSelect = async (graphName) => {
        console.log('clicked graph!', graphName);
        
        
        // Only fetch the graph data if it's a different graph
        if (selectedGraph && selectedGraph.graph_name === graphName) {
            return;
        }
        
        // Make the NodeEditorPanel spit out currently active graph
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

        setSelectedAction(null);
        setSelectedNode(null);
        actionListRef.current?.clearActiveAction();
        nodeEditorRef.current?.clearActiveNode();
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

    // Kaarel' previous. should be merged with handleActionSelect 
    const handleNodeSelect = (action) => {
        setSelectedNode(action);
    };

    const handleActionSelect = async (actionName) => {
        console.log('clicked action!', actionName);

        if (selectedNode && selectedNode.action_name === actionName) {
            return;
        }

        // Find and set the selected action
        const action = actions.find(action => action.name === actionName);
        setSelectedAction(action);

        setSelectedGraph(null);
        setSelectedNode(null);
        graphListRef.current?.clearActiveGraph();
        nodeEditorRef.current?.clearActiveNode(); // TODO: Remove blue circle around the node. React flow node still applies .selected class for the element. 
    };

    const handleNewGraph = () => {
        const newGraph = {
            graph_name: `NewGraph_${Date.now()}`,
            graph_description: "Newly created graph",
            actions: []
        };
        setGraphs([...graphs, newGraph]);
    
        setSelectedGraph(newGraph);
        setSelectedAction(null);
        setSelectedNode(null);
    
        // Set the new graph as active in GraphListPanel
        graphListRef.current?.setActiveGraph(newGraph);
    
        handleGraphSelect(newGraph.graph_name);
        console.log("new graph added, graphs: ", graphs);
    };

    useEffect(() => {
        const socket = io('http://localhost:4000');

        socket.on('graphs', (data) => {
            console.log("graphs from socket: ", data);
            setGraphs(data);
        });

        socket.on('actions', (data) => {
            console.log("actions from socket: ", data);
            setActions(data);
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
            <div className="graph-action-list-panel-div">
                <div className="graph-list-panel">
                    <GraphListPanel
                        ref={graphListRef}
                        graphsIn={graphs}
                        onGraphSelect={handleGraphSelect}
                        onStartStopClick={handleStartStopClick}
                        runtimeEnabled={runtimeEnabled}
                        onNewGraph={handleNewGraph}
                    />
                </div>

                <div className="action-list-panel">
                    <ActionListPanel
                        ref={actionListRef}
                        actionsIn={actions}
                        onActionSelect={handleActionSelect}/>
                </div>
            </div>

            <div className="node-editor-panel">
                <NodeEditorPanel
                    ref={nodeEditorRef}
                    graphDataIn={selectedGraph}
                    onUpdateGraph={handleGetCurrentGraph}
                    onNodeSelect={handleNodeSelect}/>
            </div>
            <div className="graph-info-panel">
                <GraphInfoPanel
                    graphDataIn={selectedGraph}
                    nodeDataIn={selectedNode}
                    actionDataIn={selectedAction}
                />
            </div>
        </div>
    );
};

export default ActionInterfacePage;
