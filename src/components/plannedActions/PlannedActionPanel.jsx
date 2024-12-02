// src/components/planned_actions/PlannedActionPanel.jsx

import React, { useEffect, useState } from "react";
import "./PlannedActionPanel.css";
import io from "socket.io-client";

const socket = io("http://localhost:5000"); // Adjust the URL if necessary

const PlannedActionPanel = () => {
  const resetSessionOnStart = false; // Set to true to wipe previous session memory

  // Clear sessionStorage if resetSessionOnStart is true
  if (resetSessionOnStart) {
    sessionStorage.removeItem("plannedActionQueue");
    sessionStorage.removeItem("actionStatus");
  }

  const [queue, setQueue] = useState(() => {
    const savedQueue = sessionStorage.getItem("plannedActionQueue");
    try {
      return savedQueue ? JSON.parse(savedQueue) : [];
    } catch (e) {
      console.error("Failed to parse saved queue:", e);
      return [];
    }
  });

  const [actionStatus, setActionStatus] = useState(() => {
    return sessionStorage.getItem("actionStatus") || "";
  });

  useEffect(() => {
    // Listen for queue updates from the backend
    socket.on("queue_update", (data) => {
      console.log("Received queue_update:", data);
      const newQueue = data.queue || [];
      setQueue(newQueue);
    });

    // Listen for action status updates from the backend
    socket.on("action_status", (data) => {
      console.log("Received action_status:", data);
      const newStatus = data.status || "";
      setActionStatus(newStatus);
    });

    // Clean up the event listeners on component unmount
    return () => {
      socket.off("queue_update");
      socket.off("action_status");
    };
  }, []);

  // Save queue to sessionStorage whenever it changes
  useEffect(() => {
    sessionStorage.setItem("plannedActionQueue", JSON.stringify(queue));
  }, [queue]);

  // Save actionStatus to sessionStorage whenever it changes
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
              className={`queue-item ${
                index === 0 && actionStatus ? `status-${actionStatus}` : ""
              }`}
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
