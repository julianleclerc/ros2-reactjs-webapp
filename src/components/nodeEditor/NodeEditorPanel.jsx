import React, { useState, useEffect, useCallback} from 'react';
import {
  Background,
  ReactFlow,
  ReactFlowProvider,
  useNodesState,
  useEdgesState,
  addEdge,
  useReactFlow,
  Panel,
} from '@xyflow/react';

import '@xyflow/react/dist/style.css';

const flowKey = 'example-flow';

const getNodeId = () => `randomnode_${+new Date()}`;

const NodeEditorPanel = ({ graphDataIn, onUpdateGraph }) => {

  const [activeGraph, setActiveGraph] = useState({});
  const [nodes, setNodes, onNodesChange] = useNodesState();
  const [edges, setEdges, onEdgesChange] = useEdgesState();
  const [rfInstance, setRfInstance] = useState(null);
  const { setViewport } = useReactFlow();

  const jsonToFlow = useCallback((graphJson) => {

    const new_nodes = graphJson.actions?.map((action) => ({
      id: `${action.name}_${action.instance_id}`,
      data: { label:             action.name,
              instance_id:       action.instance_id,
              type:              action.type,
              input_parameters:  action.input_parameters,
              output_parameters: action.output_parameters},
      position: action.gui_attributes.position,
    }));

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
    let activeGraphUpdated = {
        graph_name: activeGraph.graph_name,
        graph_description: activeGraph.graph_description,
        actions: []
    };

    if (rfInstance) {
        const flow = rfInstance.toObject();

        // For each "node" in flow.nodes, create a json object "umrf_node"
        flow.nodes.forEach(node => {
            let umrf_node = {
                name: node.data.label,
                instance_id: node.data.instance_id,
                type: node.data.type,
                parents: [],
                children: [],
                gui_attributes: {position: node.position}
            };

            // Find edges where the node is a source or target
            flow.edges.forEach(edge => {
                console.log('edge source_name:', edge.source_name)
                console.log('edge source_id:', edge.source_id)
                if (node.data.label === edge.source_name && node.data.instance_id === edge.source_id) {
                    // If node is a source, add to children
                    umrf_node.children.push({
                        name: edge.target_name,
                        instance_id: edge.target_id
                    });
                } else if (node.data.label === edge.target_name && node.data.instance_id === edge.target_id) {
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

    return () => {onUpdateGraph(flowToJson())}
  }, [graphDataIn, onUpdateGraph, flowToJson]);

  const onConnect = useCallback(
    (params) => setEdges((eds) => addEdge(params, eds)),
    [setEdges],
  );

  // const onSave = useCallback(() => {
  //   if (rfInstance) {
  //     const flow = rfInstance.toObject();
  //     localStorage.setItem(flowKey, JSON.stringify(flow));
  //   }
  // }, [rfInstance]);

  // const onRestore = useCallback(() => {
  //   const restoreFlow = async () => {
  //     const flow = JSON.parse(localStorage.getItem(flowKey));

  //     if (flow) {
  //       const { x = 0, y = 0, zoom = 1 } = flow.viewport;
  //       setNodes(flow.nodes || []);
  //       setEdges(flow.edges || []);
  //       setViewport({ x, y, zoom });
  //     }
  //   };

  //   restoreFlow();
  // }, [setNodes, setViewport]);

  // const onAdd = useCallback(() => {
  //   const newNode = {
  //     id: getNodeId(),
  //     data: { label: 'Added node' },
  //     position: {
  //       x: (Math.random() - 0.5) * 400,
  //       y: (Math.random() - 0.5) * 400,
  //     },
  //   };
  //   setNodes((nds) => nds.concat(newNode));
  // }, [setNodes]);

  // useEffect(() => {
  //   onRestore();

  //   return () => {onUpdateGraph()};
  // }, []);

  return (
    // <div className="node-editor-container">
      <ReactFlow
        nodes={nodes}
        edges={edges}
        onNodesChange={onNodesChange}
        onEdgesChange={onEdgesChange}
        onConnect={onConnect}
        onInit={setRfInstance}
        fitView
        fitViewOptions={{ padding: 2 }}
        style={{ backgroundColor: "#F7F9FB"}}
        >
          <Background />
      </ReactFlow>
    // </div>

    // <div className="node-editor-container"></div>
  );
};

export default ({graphDataIn, onUpdateGraph}) => (
  <ReactFlowProvider>
    <NodeEditorPanel graphDataIn={graphDataIn} onUpdateGraph={onUpdateGraph} />
  </ReactFlowProvider>
);

// export default NodeEditorPanel;
