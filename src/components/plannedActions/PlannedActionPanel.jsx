import React, { useEffect, useState } from "react";
import "./PlannedActionPanel.css";
import io from "socket.io-client";

// Initialize Socket.IO connection
const socket = io("http://localhost:4000");

// Key to store server start time
const SERVER_START_TIME_KEY = "serverStartTime";

const PlannedActionPanel = () => {
  // ----------------- State Management -----------------

  // State to track the planned action queue
  const [queue, setQueue] = useState(() => {
    const savedQueue = localStorage.getItem("plannedActionQueue");
    try {
      return savedQueue ? JSON.parse(savedQueue) : [];
    } catch (e) {
      console.error("Failed to parse saved queue:", e);
      return [];
    }
  });

  // State to track the current action status
  const [actionStatus, setActionStatus] = useState(() =>
    localStorage.getItem("actionStatus") || ""
  );

  // ----------------- useEffect Hooks -----------------

  /**
   * Checks the server start time on mount to validate localStorage data.
   */
  useEffect(() => {
    fetch("http://localhost:4000/server_start_time")
      .then((response) => response.json())
      .then((data) => {
        const serverStartTime = data.server_start_time;
        const storedServerStartTime = localStorage.getItem(SERVER_START_TIME_KEY);

        if (storedServerStartTime !== serverStartTime) {
          localStorage.clear();
          localStorage.setItem(SERVER_START_TIME_KEY, serverStartTime);

          // Reset state
          setQueue([]);
          setActionStatus("");
        }
      })
      .catch((error) => {
        console.error("Error fetching server start time:", error);
      });
  }, []);

  /**
   * Sets up Socket.IO listeners for backend updates.
   * Listens for `queue_update` and `action_status` events.
   */
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

  /**
   * Saves the `queue` state to localStorage whenever it changes.
   */
  useEffect(() => {
    localStorage.setItem("plannedActionQueue", JSON.stringify(queue));
  }, [queue]);

  /**
   * Saves the `actionStatus` state to localStorage whenever it changes.
   */
  useEffect(() => {
    localStorage.setItem("actionStatus", actionStatus);
  }, [actionStatus]);

  // -------------------- Render ------------------------

  return (
    <div className="planned-action-sub-panel">
      <div className="sub-container">
        {/* Render the queue or a "No actions planned" message */}
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
