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

    console.log("selectedGraph: ", selectedGraph);
    console.log("selectedAction: ", selectedAction);
    console.log("selectedNode: ", selectedNode);

    const handleGraphSelect = async (graphName) => {
        console.log('clicked graph!', graphName);
        
        setSelectedAction(null);
        actionListRef.current?.clearActiveAction();
        setSelectedNode(null);
        nodeEditorRef.current?.clearActiveNode();
        
        // Save current graph before switching - and WAIT for it to complete
        if (selectedGraph && selectedGraph.graph_name !== graphName && nodeEditorRef.current) {
            try {
                await nodeEditorRef.current.getCurrentGraph();
            } catch (error) {
                console.error('Error saving current graph:', error);
            }
        }
        
        // Always fetch fresh data, even for the same graph
        try {
            // Set a loading state to prevent race conditions
            setSelectedGraph(prev => ({
                ...prev,
                isLoading: true,
                graph_name: graphName // Ensure we have at least the name while loading
            }));
            
            const response = await fetch(`http://localhost:4000/api/graphs/${graphName}`);
            if (!response.ok) {
                throw new Error('Network response was not ok');
            }

            const result = await response.json();
            
            // Use functional update to avoid stale closures
            setSelectedGraph(result);
        } catch (error) {
            console.error('Error fetching data', error);
            // Reset loading state on error
            setSelectedGraph(prev => ({
                ...prev,
                isLoading: false
            }));
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
    
            setGraphs(prevGraphs => 
                prevGraphs.map(graph => 
                    graph.graph_name === updatedGraph.graph_name ? updatedGraph : graph
                )
            );
    
            // Only update selectedGraph if it's still the same graph we're editing
            setSelectedGraph(prev => 
                prev && prev.graph_name === updatedGraph.graph_name ? updatedGraph : prev
            );
    
            return updatedGraph;
        } catch (error) {
            console.error('Error updating graph:', error);
            throw error; // Rethrow to allow error handling upstream
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

        // TODO: Remove this
        //setSelectedGraph(null);
        //graphListRef.current?.clearActiveGraph();

        setSelectedNode(null);
        nodeEditorRef.current?.clearActiveNode(); // TODO: Remove blue circle around the node. React flow node still applies .selected class for the element. 
    };

    const handleNewGraph = async () => {
        const newGraph = {
            graph_name: `NewGraph_${Date.now()}`,
            graph_description: "Newly created graph",
            graph_entry: [],
            graph_exit: [],
            actions: []
        };

        try {
            const response = await fetch('http://localhost:4000/api/graphs', {
                method: 'POST',
                headers: {
                    'Content-Type': 'application/json'
                },
                body: JSON.stringify(newGraph)
            });

            if (!response.ok) {
                throw new Error('Failed to create graph in backend');
            }

            // Update frontend state after successful backend creation
            setGraphs([...graphs, newGraph]);
            setSelectedGraph(newGraph);
            setSelectedAction(null);
            setSelectedNode(null);

            // Set the new graph as active in GraphListPanel
            graphListRef.current?.setActiveGraph(newGraph);
            handleGraphSelect(newGraph.graph_name);
            console.log("new graph added, graphs: ", graphs);

        } catch (error) {
            console.error('Error creating new graph:', error);
        }
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
        if (graphs && graphs.length > 0 && !selectedGraph) {
            handleGraphSelect(graphs[0].graph_name);
            graphListRef.current?.setActiveGraph(graphs[0]);
        }

        if (graphs && selectedGraph) {
            // Store the current graph name to avoid race conditions
            const currentGraphName = selectedGraph.graph_name;
            
            // Find the updated graph data
            const updatedGraph = graphs.find(graph => graph.graph_name === currentGraphName);
            
            if (updatedGraph) {
                // Only update if we're still looking at the same graph
                // Use a functional update to avoid stale closures
                setSelectedGraph(prev => 
                    prev && prev.graph_name === currentGraphName ? updatedGraph : prev
                );
            }
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
