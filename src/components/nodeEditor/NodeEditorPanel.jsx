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

const nodeTypes = {
  turbo: SpinNode,
};

const NodeEditorPanel = forwardRef(({ graphDataIn, onUpdateGraph }, ref) => {

  const [activeGraph, setActiveGraph] = useState(null);
  const [nodes, setNodes, onNodesChange] = useNodesState();
  const [edges, setEdges, onEdgesChange] = useEdgesState();
  const [rfInstance, setRfInstance] = useState(null);
  // const { setViewport } = useReactFlow();

  useImperativeHandle(ref, () => ({
    getCurrentGraph() {
      if (activeGraph) {
        onUpdateGraph(flowToJson());
      }
    }
  }));

  const jsonToFlow = useCallback((graphJson) => {

    const new_nodes: Node<SpinNodeData>[] = graphJson.actions?.map((action) => {

      let nodeData = {
        title: action.name,
        subline: action.state && action.state !== 'UNINITIALIZED' ? action.state : '',
        instance_id: action.instance_id,
        type: action.type,
        input_parameters: action.input_parameters,
        output_parameters: action.output_parameters,
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
                gui_attributes: {position: node.position}
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

  return (
      <ReactFlow
        nodes={nodes}
        edges={edges}
        onNodesChange={onNodesChange}
        onEdgesChange={onEdgesChange}
        onConnect={onConnect}
        onInit={setRfInstance}
        nodeTypes={nodeTypes}
        fitView
        fitViewOptions={{ padding: 2 }}
        style={{ backgroundColor: "#F7F9FB"}}
        >
          <Background />
      </ReactFlow>
  );
});

export default forwardRef(({graphDataIn, onUpdateGraph}, ref) => (
  <ReactFlowProvider>
    <NodeEditorPanel ref={ref} graphDataIn={graphDataIn} onUpdateGraph={onUpdateGraph} />
  </ReactFlowProvider>
));

// export default NodeEditorPanel;
