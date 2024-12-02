import React, { useEffect, useState } from "react";
import "./PlannedActionPanel.css";
import io from "socket.io-client";

// Initialize Socket.IO connection
const socket = io("http://localhost:5000");

const PlannedActionPanel = () => {
  const resetSessionOnStart = false;

  // Clear session storage if resetSessionOnStart is enabled
  if (resetSessionOnStart) {
    sessionStorage.removeItem("plannedActionQueue");
    sessionStorage.removeItem("actionStatus");
  }

  // State management for action queue and status
  const [queue, setQueue] = useState(() => {
    const savedQueue = sessionStorage.getItem("plannedActionQueue");
    try {
      return savedQueue ? JSON.parse(savedQueue) : [];
    } catch (e) {
      console.error("Failed to parse saved queue:", e);
      return [];
    }
  });

  const [actionStatus, setActionStatus] = useState(() => sessionStorage.getItem("actionStatus") || "");

  // Listen to updates from the backend
  useEffect(() => {
    socket.on("queue_update", (data) => {
      setQueue(data.queue || []);
    });

    socket.on("action_status", (data) => {
      setActionStatus(data.status || "");
    });

    // Cleanup listeners on unmount
    return () => {
      socket.off("queue_update");
      socket.off("action_status");
    };
  }, []);

  // Save queue and action status to session storage
  useEffect(() => {
    sessionStorage.setItem("plannedActionQueue", JSON.stringify(queue));
  }, [queue]);

  useEffect(() => {
    sessionStorage.setItem("actionStatus", actionStatus);
  }, [actionStatus]);

  return (
    <div className="planned-action-panel">
      <div className="sub-container">
        {queue.length === 0 ? (
          <div className="queue-item">
            <p>No actions planned</p>
          </div>
        ) : (
          queue.map((action, index) => (
            <div
              key={index}
              className={`queue-item ${index === 0 && actionStatus ? `status-${actionStatus}` : ""}`}
            >
              <h3>Action {index + 1}</h3>
              <ul>
                {Object.entries(action).map(([key, value]) => (
                  <li key={key}>
                    <strong>{key}:</strong> {value}
                  </li>
                ))}
              </ul>
            </div>
          ))
        )}
      </div>
    </div>
  );
};

export default PlannedActionPanel;
