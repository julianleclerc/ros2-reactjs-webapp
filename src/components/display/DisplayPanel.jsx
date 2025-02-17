import React, { useState, useEffect, useRef } from "react";
import "./DisplayPanel.css";
import { io } from "socket.io-client";

// Key to store server start time
const SERVER_START_TIME_KEY = "serverStartTime";

const DisplayPanel = () => {
  // ----------------- State Management -----------------

  // State to track active feeds for camera and RViz
  const [activeFeeds, setActiveFeeds] = useState(() => {
    const savedFeeds = localStorage.getItem("activeFeeds");
    return savedFeeds ? JSON.parse(savedFeeds) : { camera: false, rviz: false };
  });

  // State to store the current camera and RViz images
  const [cameraImage, setCameraImage] = useState("");
  const [rvizImage, setRvizImage] = useState("");

  // References for WebSocket and throttling mechanisms
  const socketRef = useRef(null);
  const cameraThrottleRef = useRef(false);
  const rvizThrottleRef = useRef(false);

  // ----------------- Helper Functions -----------------

  /**
   * Toggles the state of a specific feed (camera or RViz).
   * @param {string} feed - The name of the feed to toggle.
   */
  const handleFeedToggle = (feed) => {
    setActiveFeeds((prevFeeds) => ({
      ...prevFeeds,
      [feed]: !prevFeeds[feed],
    }));
  };

  // ----------------- useEffect Hooks -----------------

  /**
   * Checks the server start time on mount to validate localStorage data or restart button.
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
          setActiveFeeds({ camera: false, rviz: false });
          setCameraImage("");
          setRvizImage("");
        }
      })
      .catch((error) => {
        console.error("Error fetching server start time:", error);
      });
  }, []);

  /**
   * Saves the `activeFeeds` state to localStorage whenever it changes.
   */
  useEffect(() => {
    localStorage.setItem("activeFeeds", JSON.stringify(activeFeeds));
  }, [activeFeeds]);

  /**                                                                               
   * Sets up WebSocket connections and listeners for camera and RViz feeds.
   * Disconnects on component unmount or when activeFeeds changes.
   */
  useEffect(() => {
    if (!socketRef.current) {
      socketRef.current = io("http://localhost:4000");

      socketRef.current.on("camera_image", (data) => {
        if (activeFeeds.camera && !cameraThrottleRef.current) {
          cameraThrottleRef.current = true;
          setCameraImage(`data:image/jpeg;base64,${data}`);
          setTimeout(() => {
            cameraThrottleRef.current = false;
          }, 200);
        }
      });

      socketRef.current.on("rviz_image", (data) => {
        if (activeFeeds.rviz && !rvizThrottleRef.current) {
          rvizThrottleRef.current = true;
          setRvizImage(`data:image/jpeg;base64,${data}`);
          setTimeout(() => {
            rvizThrottleRef.current = false;
          }, 200);
        }
      });
    }

    return () => {
      if (socketRef.current) {
        socketRef.current.disconnect();
        socketRef.current = null;
      }
    };
  }, [activeFeeds]);

  /**
   * Manages feed subscriptions based on the `activeFeeds` state.
   */
  useEffect(() => {
    if (socketRef.current) {
      if (activeFeeds.camera) {
        socketRef.current.emit("subscribe_to_camera");
      } else {
        socketRef.current.emit("unsubscribe_from_camera");
        setCameraImage("");
      }

      if (activeFeeds.rviz) {
        socketRef.current.emit("subscribe_to_rviz");
      } else {
        socketRef.current.emit("unsubscribe_from_rviz");
        setRvizImage("");
      }
    }
  }, [activeFeeds]);

  // -------------------- Render ------------------------

  // In the future, should have one button that lets us choose from list what feeds we wish to render and they should stack automatically
  const multipleFeedsActive = activeFeeds.camera && activeFeeds.rviz;

  return (
    <div className="display-sub-panel">
      {/* Button container */}
      <div className="button-display-container">
        <button
          onClick={() => handleFeedToggle("camera")}
          className={activeFeeds.camera ? "active" : ""}
        >
          Camera
        </button>
        <button
          onClick={() => handleFeedToggle("rviz")}
          className={activeFeeds.rviz ? "active" : ""}
        >
          Navigation
        </button>
      </div>

      {/* Image container */}
      <div
        className={`image-container ${
          multipleFeedsActive ? "multiple-feeds" : ""
        }`}
      >
        {activeFeeds.camera && cameraImage && (
          <img className="feed-image" src={cameraImage} alt="Camera Feed" />
        )}
        {activeFeeds.rviz && rvizImage && (
          <img className="feed-image" src={rvizImage} alt="Nav2 Feed" />
        )}
        {!activeFeeds.camera && !activeFeeds.rviz && (
          <div className="no-display-message">No display selected</div>
        )}
      </div>
    </div>
  );
};

export default DisplayPanel;
