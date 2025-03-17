import React, { useState, useEffect, useRef } from 'react';
import io from 'socket.io-client';
import GraphListPanel from "../../components/graphList/GraphListPanel.jsx";
import ActionListPanel from "../../components/actionList/ActionListPanel.jsx";
import NodeEditorPanel from "../../components/nodeEditor/NodeEditorPanel.jsx";
import GraphInfoPanel from "../../components/graphInfo/GraphInfoPanel.jsx";
import "./ActionInterfacePage.css";

const ActionInterfacePage = () => {
    // Data states
    const [graphs, setGraphs] = useState(null);
    const [actions, setActions] = useState(null);
    const [runtimeEnabled, setRuntimeEnabled] = useState(false);
    
    // Add a separate state for the active graph
    const [activeGraph, setActiveGraph] = useState(null);
    
    // Selection state - for info panel only
    const [selectedElement, setSelectedElement] = useState({
        type: null, // 'graph', 'node', 'action'
        data: null
    });
    
    // Track active IDs for visual selection in panels
    const [activeGraphId, setActiveGraphId] = useState(null);
    const [activeActionId, setActiveActionId] = useState(null);
    const [activeNodeId, setActiveNodeId] = useState(null);
    
    const [isNewAction, setIsNewAction] = useState(false);
    
    // Only keep refs that are truly necessary
    const nodeEditorRef = useRef();
    const graphListRef = useRef();
    const actionListRef = useRef();

    console.log("graphs: ", graphs);
    console.log("actions: ", actions);

    console.log("selectedElement: ", selectedElement);

    const handleGraphSelect = async (graphName) => {
        console.log('clicked graph!', graphName);
        
        // Clear other active selections visually
        setActiveActionId(null);
        setActiveNodeId(null);
        nodeEditorRef.current?.clearActiveNode();
        
        // Save current graph before switching
        if (activeGraphId && activeGraphId !== graphName && nodeEditorRef.current) {
            try {
                await nodeEditorRef.current.getCurrentGraph();
            } catch (error) {
                console.error('Error saving current graph:', error);
            }
        }
        
        try {
            // Set loading state
            setActiveGraphId(graphName);
            
            const response = await fetch(`http://localhost:4000/api/graphs/${graphName}`);
            if (!response.ok) {
                throw new Error('Network response was not ok');
            }

            const result = await response.json();
            
            // Update both the active graph and the selected element
            setActiveGraph(result);
            setSelectedElement({
                type: 'graph',
                data: result
            });
        } catch (error) {
            console.error('Error fetching data', error);
            setActiveGraphId(null);
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
    
            // Update both states
            setActiveGraph(updatedGraph);
            
            // Only update selectedElement if it's still showing the graph
            setSelectedElement(prev => 
                prev && prev.type === 'graph' && prev.data.graph_name === updatedGraph.graph_name ? {
                    type: 'graph',
                    data: updatedGraph
                } : prev
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

    const handleNodeSelect = (nodeData) => {
        if (!nodeData) {
            setActiveNodeId(null);
            // If no node is selected, show the graph instead
            if (activeGraphId) {
                const graph = graphs.find(g => g.graph_name === activeGraphId);
                setSelectedElement({
                    type: 'graph',
                    data: graph
                });
            }
            return;
        }
        
        setActiveNodeId(nodeData.instance_id);
        // Only update what's shown in the info panel, not the active graph
        setSelectedElement({
            type: 'node',
            data: nodeData
        });
    };

    const handleActionSelect = async (actionName) => {
        console.log('clicked action!', actionName);

        // Clear node selection
        setActiveNodeId(null);
        nodeEditorRef.current?.clearActiveNode();
        
        // Find and set the selected action
        const action = actions.find(action => action.name === actionName);
        if (!action) return;
        
        setActiveActionId(actionName);
        setSelectedElement({
            type: 'action',
            data: action
        });
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
            setActiveGraphId(newGraph.graph_name);
            setActiveActionId(null);
            setActiveNodeId(null);

            // Set the new graph as active in GraphListPanel
            graphListRef.current?.setActiveGraph(newGraph);
            handleGraphSelect(newGraph.graph_name);
            console.log("new graph added, graphs: ", graphs);

        } catch (error) {
            console.error('Error creating new graph:', error);
        }
    };

    const handleNewAction = () => {
        const newAction = {
            name: `NewAction_${Date.now()}`,
            description: "Newly created action",
            parameters: {}
        };

        setActions([...actions, newAction]);
        setActiveActionId(newAction.name);
        setIsNewAction(true);
        setSelectedElement({
            type: 'action',
            data: newAction
        });
    };

    const handleActionGenerated = (generatedAction) => {
        setIsNewAction(false);
        
        // If we have the generated action data, update it in the UI
        if (generatedAction) {
            // Update actions list (this will be handled by socket in real implementation)
            // For immediate feedback, we can update locally:
            const updatedActions = [...actions, generatedAction];
            setActions(updatedActions);
            
            // Select the newly generated action
            setActiveActionId(generatedAction.name);
            actionListRef.current?.setActiveAction(generatedAction);
            
            // Update the selected element
            setSelectedElement({
                type: 'action',
                data: generatedAction
            });
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
        if (graphs && graphs.length > 0 && !activeGraphId) {
            const firstGraph = graphs[0];
            setActiveGraphId(firstGraph.graph_name);
            setActiveGraph(firstGraph);
        }

        if (graphs && activeGraphId) {
            // Store the current graph name to avoid race conditions
            const currentGraphName = activeGraphId;
            
            // Find the updated graph data
            const updatedGraph = graphs.find(graph => graph.graph_name === currentGraphName);
            
            if (updatedGraph) {
                // Update the active graph
                setActiveGraph(updatedGraph);
                
                // Only update selectedElement if it's showing the graph
                setSelectedElement(prev => 
                    prev && prev.type === 'graph' && prev.data.graph_name === currentGraphName ? {
                        type: 'graph',
                        data: updatedGraph
                    } : prev
                );
            }
        }
    }, [graphs]);

    return (
        <div className="action-interface-page">
            <div className="graph-action-list-panel-div">
                <div className="graph-list-panel">
                    <GraphListPanel
                        graphs={graphs}
                        activeGraphId={activeGraphId}
                        onGraphSelect={handleGraphSelect}
                        onStartStopClick={handleStartStopClick}
                        runtimeEnabled={runtimeEnabled}
                        onNewGraph={handleNewGraph}
                    />
                </div>

                <div className="action-list-panel">
                    <ActionListPanel
                        actions={actions}
                        activeActionId={activeActionId}
                        onActionSelect={handleActionSelect}
                        selectedGraph={selectedElement.type === 'graph' ? selectedElement.data : null}
                        onNewAction={handleNewAction}
                    />
                </div>
            </div>

            <div className="node-editor-panel">
                <NodeEditorPanel
                    ref={nodeEditorRef}
                    graphDataIn={activeGraph}
                    onUpdateGraph={handleGetCurrentGraph}
                    onNodeSelect={handleNodeSelect}/>
            </div>
            <div className="graph-info-panel">
                <GraphInfoPanel
                    selectedElement={selectedElement}
                    isNewAction={isNewAction}
                    onActionGenerated={handleActionGenerated}
                />
            </div>
        </div>
    );
};

export default ActionInterfacePage;
