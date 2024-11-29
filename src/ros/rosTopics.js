import ROSLIB from "roslib";
import ros from "./rosService";

// ROS Services
export const chatService = new ROSLIB.Service({
    ros: ros,
    name: "/chat_service",
    serviceType: "interfaces/srv/Chat",
});

// ROS Topics
export const promptResponsePublisher = new ROSLIB.Topic({
    ros: ros,
    name: "/prompt_response",
    messageType: "std_msgs/String",
});

export const promptRequestSubscriber = new ROSLIB.Topic({
    ros: ros,
    name: "/prompt_request",
    messageType: "std_msgs/String",
});

export const jsonQueueListener = new ROSLIB.Topic({
    ros: ros,
    name: "/json_queue",
    messageType: "std_msgs/String",
});

export const actionStatusSubscriber = new ROSLIB.Topic({
    ros: ros,
    name: "/action_status",
    messageType: "std_msgs/String",
});

export const promptInfoSubscriber = new ROSLIB.Topic({
    ros: ros,
    name: "/prompt_info",
    messageType: "std_msgs/String",
});

export const promptAlertSubscriber = new ROSLIB.Topic({
    ros: ros,
    name: "/prompt_alert",
    messageType: "std_msgs/String",
});
