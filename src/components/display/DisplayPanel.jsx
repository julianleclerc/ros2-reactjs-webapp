import React, { useState } from "react";
import "./DisplayPanel.css";
import ROSLIB from "roslib";


const DisplayPanel = () => {
    // ------------------------- State Management -------------------------
    
    // State to hold the current image source
    const [image, setImage] = useState("../static/images/image1.png");

    // ------------------------- Helper Functions -------------------------

    // Handle manual image change and unsubscribe from the ROS topic
    const handleImageChange = (src) => {
        setImage(src);
    };

    // Subscribe to the RViz ROS topic to receive image updates
    const subscribeToRviz = (src) => {
        setImage(src);
    };

    // ------------------------- Render UI -------------------------

    return (
        <div className="display-panel">
            {/* Button container for switching views */}
            <div className="button-container">
                <button onClick={() => handleImageChange("../static/images/image1.png")}>
                    Camera
                </button>
                <button onClick={subscribeToRviz}>
                    Nav2
                </button>
            </div>

            {/* Display the image */}
            <img id="robot-image" src={image} alt="Robot Display" />
        </div>
    );
};

export default DisplayPanel;
