import React, { useState, useEffect, useRef } from "react";
import "./ChatPanel.css";
import io from "socket.io-client";

// Socket.IO configuration
const socket = io("http://localhost:5000");

const ChatPanel = () => {
  // ---------------------- States -----------------------
  const [tabs, setTabs] = useState(() => loadFromSession("chatTabs", [{ name: "Global", namespace: null }]));
  const [activeTab, setActiveTab] = useState("Global");
  const [messages, setMessages] = useState(() => loadFromSession("chatMessages", { Global: [] }));
  const [input, setInput] = useState("");
  const chatLogRef = useRef(null);

  // ------------------ Configuration -------------------
  const resetSessionOnStart = true; // Clears session storage on boot if true
  if (resetSessionOnStart) sessionStorage.clear();

  // ------------------- Effects ------------------------
  useEffect(() => saveToSession("chatTabs", tabs), [tabs]);
  useEffect(() => saveToSession("chatMessages", messages), [messages]);
  useEffect(() => setupSocketListener(), []);
  useEffect(() => autoScrollToBottom(), [messages, activeTab]);

  // ----------------- Helper Functions -----------------

  // Load data from session storage or return default value
  function loadFromSession(key, defaultValue) {
    const savedData = sessionStorage.getItem(key);
    return savedData ? JSON.parse(savedData) : defaultValue;
  }

  // Save data to session storage
  function saveToSession(key, value) {
    sessionStorage.setItem(key, JSON.stringify(value));
  }

  // Set up Socket.IO listener for incoming messages
  function setupSocketListener() {
    socket.on("ros_message", (data) => {
      updateMessages("Global", {
        user: data.user,
        message: data.message,
        type: data.type || "default",
      });
    });

    return () => socket.off("ros_message");
  }

  // Auto-scroll the chat log to the bottom
  function autoScrollToBottom() {
    if (chatLogRef.current) {
      chatLogRef.current.scrollTop = chatLogRef.current.scrollHeight;
    }
  }

  // Update the messages for a specific tab
  function updateMessages(tabName, newMessage) {
    setMessages((prevMessages) => ({
      ...prevMessages,
      [tabName]: [...(prevMessages[tabName] || []), newMessage],
    }));
  }

  // Handle sending messages to the backend
  async function sendMessageToBackend(request) {
    try {
      const response = await fetch("http://localhost:5000/send_message", {
        method: "POST",
        headers: { "Content-Type": "application/json" },
        body: JSON.stringify(request),
      });

      if (!response.ok) throw new Error("Failed to send message");
      const data = await response.json();
      updateMessages(activeTab, { user: "ROS", message: data.response, type: "default" });
    } catch (error) {
      console.error("Error:", error.message);
      updateMessages(activeTab, {
        user: "System",
        message: "Error: Unable to reach the backend server.",
        type: "alert",
      });
    }
  }

  // ---------------- Event Handlers --------------------

  // Handle sending a message
  const handleSend = async () => {
    if (!input.trim()) return;

    // Add the user's message to the current tab
    updateMessages(activeTab, { user: "You", message: input.trim(), type: "default" });

    // Prepare and send the request to the backend
    const namespace = tabs.find((tab) => tab.name === activeTab)?.namespace || null;
    const request = { message: input.trim(), namespace };
    setInput("");
    await sendMessageToBackend(request);
  };

  // Handle adding a new tab
  const handleAddTab = () => {
    const namespace = prompt("Enter the robot namespace:");
    if (!namespace) return;

    if (tabs.some((tab) => tab.name === namespace)) {
      alert("A tab with this namespace already exists.");
      return;
    }

    setTabs((prevTabs) => [...prevTabs, { name: namespace, namespace }]);
    setMessages((prevMessages) => ({ ...prevMessages, [namespace]: [] }));
  };

  // Handle sending a message when 'Enter' key is pressed
  const handleKeyDown = (e) => {
    if (e.key === "Enter") {
      e.preventDefault();
      handleSend();
    }
  };

  // Placeholder for voice prompt activation
  const handleVoicePrompt = () => alert("Voice Prompt Activated! (🎙️)");

  // -------------------- Render ------------------------
  return (
    <div className="chat-sub-panel">
      <div className="chat-container">

        {/* Tabs for different namespaces */}
        <div className="tab-container">
          <button className="add-tab-button" onClick={handleAddTab}>+</button>
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

        {/* Chat messages display */}
        <div className="chat-log" ref={chatLogRef}>
          {(messages[activeTab] || []).map((chat, idx) => (
            <p key={idx} className={`chat-message ${chat.type || ""}`}>
              <strong>{chat.user}:</strong> {chat.message}
            </p>
          ))}
        </div>

        {/* Input field and controls */}
        <div className="chat-input-container">
          <input
            type="text"
            value={input}
            onChange={(e) => setInput(e.target.value)}
            onKeyDown={handleKeyDown}
            placeholder="Type your message here..."
          />
          <button onClick={handleVoicePrompt} className="voice-prompt-button">🎙️</button>
          <button onClick={handleSend}>Send</button>
        </div>
      </div>
    </div>
  );
};

export default ChatPanel;
