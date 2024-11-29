import ROSLIB from "roslib";

// ROS initialization
const ros = new ROSLIB.Ros({
    url: "ws://localhost:9090", // Update to your ROS bridge address
});

// Connection Handlers
ros.on("connection", () => {
    console.log("Connected to ROS bridge.");
});

ros.on("error", (error) => {
    console.error("Error connecting to ROS bridge:", error);
});

ros.on("close", () => {
    console.log("Connection to ROS bridge closed.");
});

// Export the ROS instance for use in other components
export default ros;
