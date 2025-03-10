import React, { useState, useEffect, useCallback, forwardRef, useImperativeHandle} from 'react';
import {
  Background,
  ReactFlow,
  ReactFlowProvider,
  useNodesState,
  useEdgesState,
  addEdge,
  // useReactFlow,
  // Panel,
} from '@xyflow/react';

import SpinNode, { type SpinNodeData } from './SpinNode.tsx';

// import '@xyflow/react/dist/style.css';

import '@xyflow/react/dist/base.css';
// import "./NodeEditorPanel.css";
import "./SpinNode.css";
import "./SelectedNode.css";

import { useDnD } from "../../components/actionList/DnDContext.jsx";

const nodeTypes = {
  turbo: SpinNode,
};

const NodeEditorPanel = forwardRef(({ graphDataIn, onUpdateGraph, onNodeSelect }, ref) => {

  const [activeGraph, setActiveGraph] = useState(null);
  const [nodes, setNodes, onNodesChange] = useNodesState();
  const [edges, setEdges, onEdgesChange] = useEdgesState();
  const [rfInstance, setRfInstance] = useState(null);
  const [selectedNodeId, setSelectedNodeId] = useState(null);
  // const { setViewport } = useReactFlow();
  const [type] = useDnD();

  useImperativeHandle(ref, () => ({
    getCurrentGraph() {
      if (activeGraph) {
        onUpdateGraph(flowToJson());
      }
    },
    clearActiveNode: () => {
      console.log("Clearing active node", selectedNodeId);
      setSelectedNodeId(null);
    }
  }));

  const jsonToFlow = useCallback((graphJson) => {

    const new_nodes: Node<SpinNodeData>[] = graphJson.actions?.map((action) => {

      console.log("Json to Flow action: ", action);
      console.log("Action actor: ", action);
      //console.log("Action input parameters: ", action.input_parameters);
      let nodeData = {
        title: action.name,
        subline: action.state && action.state !== 'UNINITIALIZED' ? action.state : '',
        instance_id: action.instance_id,
        type: action.type,
        input_parameters: action.input_parameters,
        output_parameters: action.output_parameters,
        actor: action.actor,
      };

      // Conditionally add state field
      if (action.state) {
          nodeData.state = action.state;
      }

      return {
        id: `${action.name}_${action.instance_id}`,
        data: nodeData,
        position: action.gui_attributes.position,
        type: 'turbo'
      };
    });

    const new_edges = graphJson.actions.flatMap(action =>
      action.children?.map(child => ({
        id: `${action.name}_${action.instance_id} to ${child.name}_${child.instance_id}`,
        source: `${action.name}_${action.instance_id}`,
        target: `${child.name}_${child.instance_id}`,

        // Needed for converting back to UMRF graph
        source_name: action.name,
        source_id:   action.instance_id,
        target_name: child.name,
        target_id:   child.instance_id,
      })) || []
    );

    setActiveGraph(graphJson)
    setNodes(new_nodes)
    setEdges(new_edges)
  }, [setActiveGraph, setNodes, setEdges]);

  const flowToJson = useCallback(() => {
    console.log("flowToJson: ", activeGraph.graph_name)

    let activeGraphUpdated = {
        graph_name: activeGraph.graph_name,
        graph_description: activeGraph.graph_description,
        actions: []
    };

    if (activeGraph.graph_state){
      activeGraphUpdated.graph_state = activeGraph.graph_state;
    }

    if (rfInstance) {
        const flow = rfInstance.toObject();

        // For each "node" in flow.nodes, create a json object "umrf_node"
        flow.nodes.forEach(node => {
            let umrf_node = {
                name: node.data.title,
                instance_id: node.data.instance_id,
                type: node.data.type,
                parents: [],
                children: [],
                gui_attributes: {position: node.position},
                input_parameters: node.data.input_parameters,
                output_parameters: node.data.output_parameters,
                actor: node.data.actor
            };

            if (node.data.state){
              umrf_node.state = node.data.state
            }

            // Find edges where the node is a source or target
            flow.edges.forEach(edge => {
                if (node.data.title === edge.source_name && node.data.instance_id === edge.source_id) {
                    // If node is a source, add to children
                    umrf_node.children.push({
                        name: edge.target_name,
                        instance_id: edge.target_id
                    });
                } else if (node.data.title === edge.target_name && node.data.instance_id === edge.target_id) {
                    // If node is a target, add to parents
                    umrf_node.parents.push({
                        name: edge.source_name,
                        instance_id: edge.source_id
                    });
                }
            });

            // Add "umrf_node" to "activeGraphUpdated.actions" array
            activeGraphUpdated.actions.push(umrf_node);
        });
    }

    return activeGraphUpdated;
  }, [activeGraph, rfInstance]);

  useEffect(() => {
    if (graphDataIn) {
      jsonToFlow(graphDataIn);
    }
  }, [graphDataIn, jsonToFlow]);

  const onConnect = useCallback(
    (params) => setEdges((eds) => addEdge(params, eds)),
    [setEdges],
  );

  const handleNodeClick = (node) => {
    console.log("Node clicked, node: ", node);
    setSelectedNodeId(node.id);
    onNodeSelect(node.data);
  };
  const onDragOver = useCallback((event) => {
    event.preventDefault();
    event.dataTransfer.dropEffect = 'move';
  }, []);

  const onDrop = useCallback(
    (event) => {
      event.preventDefault();
      console.log('Drop event triggered');

      const dragType = type || 'turbo';

      console.log('Type at drop:', dragType);
      if (!dragType) return;

      const actionName = event.dataTransfer.getData('actionName');
      console.log('Action Name at drop:', actionName);
      if (!actionName) return;

      // Get the flow container's bounds
      const reactFlowBounds = event.currentTarget.getBoundingClientRect();
      const position = {
        x: event.clientX - reactFlowBounds.left,
        y: event.clientY - reactFlowBounds.top
      };

      // Create new UMRF node
      const newNode = {
        name: actionName,
        instance_id: Date.now(), // Use timestamp as temporary unique ID
        type: "sync", // Default type
        input_parameters: {}, // Will be populated from action template
        parents: [],
        children: [],
        gui_attributes: {
          position: position
        }
      };

      // Update the graph structure
      const updatedGraph = {
        ...activeGraph,
        actions: [...(activeGraph?.actions || []), newNode]
      };

      setActiveGraph(updatedGraph);
      onUpdateGraph(updatedGraph);

      // Add node to flow
      const flowNode = {
        id: `${actionName}_${newNode.instance_id}`,
        type: dragType,
        position,
        data: {
          title: actionName,
          instance_id: newNode.instance_id,
          type: "sync",
          input_parameters: {},
          output_parameters: {},
        }
      };

      setNodes((nds) => Array.isArray(nds) ? [...nds, flowNode] : [flowNode]);
    },
    [type, setNodes, activeGraph, onUpdateGraph]
  );

  return (
      <ReactFlow
        nodes={nodes}
        edges={edges}
        onNodesChange={onNodesChange}
        onEdgesChange={onEdgesChange}
        onConnect={onConnect}
        onInit={setRfInstance}
        nodeTypes={nodeTypes}
        onDrop={onDrop}
        onDragOver={onDragOver}
        fitView
        fitViewOptions={{ padding: 2 }}
        style={{ backgroundColor: "#F7F9FB"}}
        onNodeClick={(event, node) => handleNodeClick(node)}
        >
          <Background />
      </ReactFlow>
  );
});

export default forwardRef(({graphDataIn, onUpdateGraph, onNodeSelect}, ref) => (
  <ReactFlowProvider>
    <NodeEditorPanel ref={ref} graphDataIn={graphDataIn} onUpdateGraph={onUpdateGraph} onNodeSelect={onNodeSelect} />
  </ReactFlowProvider>
));

// export default NodeEditorPanel;
