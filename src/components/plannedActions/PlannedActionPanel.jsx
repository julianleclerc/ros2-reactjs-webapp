import React, { useEffect, useState } from "react";
import "./PlannedActionPanel.css";
import io from "socket.io-client";

// Initialize Socket.IO connection
const socket = io("http://localhost:4000");

// Recursive helper to render a parameter key-value pair in a compact, nested style.
const renderParameter = (key, value) => {
  if (
    value &&
    typeof value === "object" &&
    "pvf_type" in value &&
    "pvf_value" in value
  ) {
    // For parameters with the pvf structure, display only the pvf_value.
    return (
      <li key={key}>
        <strong>{key}:</strong> {value.pvf_value}
      </li>
    );
  } else if (value && typeof value === "object") {
    // For nested objects, recursively render their keys.
    return (
      <li key={key}>
        <strong>{key}:</strong>
        <ul>
          {Object.entries(value).map(([nestedKey, nestedValue]) =>
            renderParameter(nestedKey, nestedValue)
          )}
        </ul>
      </li>
    );
  } else {
    return (
      <li key={key}>
        <strong>{key}:</strong> {value}
      </li>
    );
  }
};

const renderParameters = (params) => (
  <ul>
    {Object.entries(params).map(([key, value]) => renderParameter(key, value))}
  </ul>
);

// A component for a single action item.
const ActionItem = ({ nodeData, status }) => {
  const [showParams, setShowParams] = useState(false);

  const toggleParams = () => {
    setShowParams((prev) => !prev);
  };

  return (
    <div className={`planned-action-item`}>

      {/* Action Status */}
      <div className={'status-action-item'}>{nodeData.subline}</div>

      {/* Header: Name on the left, Status on the right */}
      <div className="planned-action-item-header">
        <div className="action-title">{nodeData.title}</div>
      </div>

      {/* Toggle to show/hide parameters */}
      <div className="parameter-toggle" onClick={toggleParams}>
        {showParams ? "Hide input parameters" : "Show input parameters"}
      </div>

      {showParams && (
        <div className="parameters-container">
          {nodeData.input_parameters
            ? renderParameters(nodeData.input_parameters)
            : <p>No input parameters</p>}
        </div>
      )}
    </div>
  );
};

const PlannedActionPanel = () => {
  const [actions, setActions] = useState([]);
  const [actionStatus, setActionStatus] = useState("");

  useEffect(() => {
    console.log("[PlannedActionPanel] Component mounted. Connecting to socket events...");

    // Listen for the 'graphs' event from the backend.
    socket.on("graphs", (data) => {
      console.log("[PlannedActionPanel] Received 'graphs' event:", data);
      if (data && Array.isArray(data) && data.length > 0) {
        // For demo purposes, use the first graph in the list.
        const selectedGraph = data[0];
        console.log("[PlannedActionPanel] Selected graph:", selectedGraph);

        if (selectedGraph.actions) {
          console.log("[PlannedActionPanel] Found actions:", selectedGraph.actions);
          setActions(selectedGraph.actions);
        } else {
          console.warn("[PlannedActionPanel] No actions found in the selected graph:", selectedGraph);
          setActions([]);
        }
      } else {
        console.warn("[PlannedActionPanel] Received 'graphs' event with no graphs:", data);
        setActions([]);
      }
    });

    // Listen for the 'action_status' event.
    socket.on("action_status", (data) => {
      console.log("[PlannedActionPanel] Received 'action_status' event:", data);
      if (data && data.status) {
        setActionStatus(data.status);
      } else {
        console.warn("[PlannedActionPanel] No status found in received data:", data);
      }
    });

    return () => {
      console.log("[PlannedActionPanel] Cleaning up socket listeners...");
      socket.off("graphs");
      socket.off("action_status");
    };
  }, []);

  // Transform each action into a nodeData-like object.
  const transformedActions = actions.map((action, index) => {
    const nodeData = {
      title: action.name,
      subline: action.state,
      instance_id: action.instance_id,
      type: action.type,
      input_parameters: action.input_parameters,
      output_parameters: action.output_parameters,
    };
    return nodeData;
  });

  return (
    <div className="planned-action-sub-panel">
      <div className="planned-action-sub-container">
        {transformedActions.length === 0 ? (
          <div className="planned-action-item">
            <p>No actions planned</p>
          </div>
        ) : (
          transformedActions.map((nodeData, index) => (
            <ActionItem
              key={nodeData.instance_id || index}
              nodeData={nodeData}
              // Only the first action gets the status badge
              status={index === 0 ? actionStatus : null}
            />
          ))
        )}
      </div>
    </div>
  );
};

export default PlannedActionPanel;
