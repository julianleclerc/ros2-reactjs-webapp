import React, { useState } from "react";
import "./ChatPanel.css";
import { chatService } from "../../ros/rosTopics";

const ChatPanel = () => {
    const [tabs, setTabs] = useState([{ name: "Global", namespace: null }]); // Default tab
    const [activeTab, setActiveTab] = useState("Global");
    const [messages, setMessages] = useState({ Global: [] });
    const [input, setInput] = useState("");

    const handleSend = () => {
        if (!input.trim()) return; // Prevent sending empty messages

        // Add message to the active tab's message log
        setMessages((prevMessages) => ({
            ...prevMessages,
            [activeTab]: [...prevMessages[activeTab], { user: "You", message: input }],
        }));

        const request = { message: input };
        chatService.callService(request, (response) => {
            setMessages((prevMessages) => ({
                ...prevMessages,
                [activeTab]: [
                    ...prevMessages[activeTab],
                    { user: "ROS", message: response.response },
                ],
            }));
        });

        setInput(""); // Clear the input field after sending
    };

    const handleVoicePrompt = () => {
        alert("Voice Prompt Activated! (🎙️)"); // Placeholder for voice input logic
    };

    const handleKeyDown = (e) => {
        if (e.key === "Enter") {
            e.preventDefault(); // Prevent default behavior (e.g., adding a newline)
            handleSend();
        }
    };

    const handleAddTab = () => {
        const namespace = prompt("Enter the robot namespace:");
        if (!namespace) return;

        // Check if a tab with the same namespace already exists
        if (tabs.some((tab) => tab.name === namespace)) {
            alert("A tab with this namespace already exists.");
            return;
        }

        // Add the new tab and initialize its message log
        setTabs((prevTabs) => [...prevTabs, { name: namespace, namespace }]);
        setMessages((prevMessages) => ({
            ...prevMessages,
            [namespace]: [],
        }));
    };

    return (
        <div className="chat-panel">
            <div className="chat-container">
                {/* Tabs */}
                <div className="tab-container">
                    <button className="add-tab-button" onClick={handleAddTab}>
                        +
                    </button>
                    <div className="tabs-scrollable">
                        {tabs.map((tab) => (
                            <button
                                key={tab.name}
                                className={`tab-button ${activeTab === tab.name ? "active-tab" : ""}`}
                                onClick={() => setActiveTab(tab.name)}
                            >
                                {tab.name}
                            </button>
                        ))}
                    </div>
                </div>

                {/* Chat Log */}
                <div className="chat-log">
                    {messages[activeTab]?.map((chat, idx) => (
                        <p key={idx}>
                            <strong>{chat.user}:</strong> {chat.message}
                        </p>
                    ))}
                </div>

                {/* Chat Input */}
                <div className="chat-input-container">
                    <input
                        type="text"
                        value={input}
                        onChange={(e) => setInput(e.target.value)}
                        onKeyDown={handleKeyDown} // Listen for Enter key
                        placeholder="Type your message here..."
                    />
                    <button onClick={handleVoicePrompt} className="voice-prompt-button">
                        🎙️
                    </button>
                    <button onClick={handleSend}>Send</button>
                </div>
            </div>
        </div>
    );
};

export default ChatPanel;
