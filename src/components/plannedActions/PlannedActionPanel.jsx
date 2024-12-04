import React, { useEffect, useState } from "react";
import "./PlannedActionPanel.css";
import io from "socket.io-client";

// Initialize Socket.IO connection
const socket = io("http://localhost:5000");

// Key to store server start time
const SERVER_START_TIME_KEY = "serverStartTime";

const PlannedActionPanel = () => {
  const [queue, setQueue] = useState(() => {
    const savedQueue = localStorage.getItem("plannedActionQueue");
    try {
      return savedQueue ? JSON.parse(savedQueue) : [];
    } catch (e) {
      console.error("Failed to parse saved queue:", e);
      return [];
    }
  });

  const [actionStatus, setActionStatus] = useState(() => localStorage.getItem("actionStatus") || "");

  // Check server start time on mount
  useEffect(() => {
    fetch("http://localhost:5000/server_start_time")
      .then((response) => response.json())
      .then((data) => {
        const serverStartTime = data.server_start_time;
        console.log("Received server start time:", serverStartTime);
        const storedServerStartTime = localStorage.getItem(SERVER_START_TIME_KEY);
        console.log("Stored server start time:", storedServerStartTime);

        if (storedServerStartTime !== serverStartTime) {
          console.log("Server start time mismatch. Clearing local storage.");
          localStorage.clear();
          localStorage.setItem(SERVER_START_TIME_KEY, serverStartTime);

          // Reset state
          setQueue([]);
          setActionStatus("");
        } else {
          console.log("Server start time matches. No action needed.");
        }
      })
      .catch((error) => {
        console.error("Error fetching server start time:", error);
      });
  }, []);

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

  // Save queue and action status to local storage
  useEffect(() => {
    localStorage.setItem("plannedActionQueue", JSON.stringify(queue));
  }, [queue]);

  useEffect(() => {
    localStorage.setItem("actionStatus", actionStatus);
  }, [actionStatus]);

  return (
    <div className="planned-action-sub-panel">
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
