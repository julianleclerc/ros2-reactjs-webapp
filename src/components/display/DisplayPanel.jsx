import React, { useState, useEffect, useRef } from "react";
import "./DisplayPanel.css";
import { io } from "socket.io-client";

const DisplayPanel = () => {
  const [activeFeeds, setActiveFeeds] = useState(() => {
    const savedFeeds = localStorage.getItem("activeFeeds");
    return savedFeeds ? JSON.parse(savedFeeds) : { camera: false, rviz: false };
  });
  const [cameraImage, setCameraImage] = useState("");
  const [rvizImage, setRvizImage] = useState("");
  const socketRef = useRef(null);
  const cameraThrottleRef = useRef(false);
  const rvizThrottleRef = useRef(false);

  useEffect(() => {
    localStorage.setItem("activeFeeds", JSON.stringify(activeFeeds));
  }, [activeFeeds]);

  useEffect(() => {
    if (!socketRef.current) {
      socketRef.current = io("http://localhost:5000");

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

  const handleFeedToggle = (feed) => {
    setActiveFeeds((prevFeeds) => ({
      ...prevFeeds,
      [feed]: !prevFeeds[feed],
    }));
  };

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
