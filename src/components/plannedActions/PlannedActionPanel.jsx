import React, { useEffect, useState } from "react";
import "./PlannedActionPanel.css";
import { jsonQueueListener } from "../../ros/rosTopics";

const PlannedActionPanel = () => {
    const [queue, setQueue] = useState([]);

    useEffect(() => {
        // Subscribe to the /json_queue topic
        jsonQueueListener.subscribe((message) => {
            try {
                const data = JSON.parse(message.data);
                setQueue(data.queue || []);
            } catch (e) {
                console.error("Failed to parse /json_queue message:", e);
            }
        });

        // Cleanup subscription on component unmount
        return () => jsonQueueListener.unsubscribe();
    }, []);

    return (
        <div className="planned-action-panel">
            <div className="sub-container">
                {queue.length === 0 ? (
                    <div className="queue-item">
                        <p>No actions planned</p>
                    </div>
                ) : (
                    queue.map((action, index) => (
                        <div key={index} className="queue-item">
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
