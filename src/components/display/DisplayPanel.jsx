import React, { useState } from "react";
import "./DisplayPanel.css";
import ROSLIB from "roslib";
import ros from "../../ros/rosService";

const rvizDisplayTopic = new ROSLIB.Topic({
    ros: ros,
    name: "/rviz_display_base64",
    messageType: "std_msgs/String",
});

const DisplayPanel = () => {
    const [image, setImage] = useState("../static/images/image1.png");

    const handleImageChange = (src) => {
        setImage(src);
        rvizDisplayTopic.unsubscribe();
    };

    const subscribeToRviz = () => {
        rvizDisplayTopic.subscribe((message) => {
            setImage(`data:image/jpeg;base64,${message.data}`);
        });
    };

    return (
        <div className="display-panel">
            <div className="button-container">
                <button onClick={() => handleImageChange("../static/images/image1.png")}>Camera</button>
                <button onClick={subscribeToRviz}>Nav2</button>
            </div>
            <img id="robot-image" src={image} alt="Robot Display" />
        </div>
    );
};

export default DisplayPanel;
