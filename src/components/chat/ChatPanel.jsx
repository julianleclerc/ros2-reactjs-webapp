// src/components/chat/ChatPanel.jsx

import React, { useState, useEffect, useRef, useCallback } from "react";
import "./ChatPanel.css";
import io from "socket.io-client";

// Socket.IO configuration
const socket = io("http://localhost:5000");

// Key to store server start time
const SERVER_START_TIME_KEY = "serverStartTime";

const ChatPanel = () => {
  const [tabs, setTabs] = useState(() =>
    loadFromStorage("chatTabs", [{ name: "Global", namespace: null }])
  );
  const [activeTab, setActiveTab] = useState("Global");
  const [messages, setMessages] = useState(() =>
    loadFromStorage("chatMessages", { Global: [] })
  );
  const [input, setInput] = useState("");
  const chatLogRef = useRef(null);

  // ----------------- Helper Functions -----------------

  function loadFromStorage(key, defaultValue) {
    const savedData = localStorage.getItem(key);
    return savedData ? JSON.parse(savedData) : defaultValue;
  }

  function saveToStorage(key, value) {
    localStorage.setItem(key, JSON.stringify(value));
  }

  function autoScrollToBottom() {
    if (chatLogRef.current) {
      chatLogRef.current.scrollTop = chatLogRef.current.scrollHeight;
    }
  }

  function updateMessages(tabName, newMessage) {
    setMessages((prevMessages) => ({
      ...prevMessages,
      [tabName]: [...(prevMessages[tabName] || []), newMessage],
    }));
  }

  async function sendMessageToBackend(request) {
    try {
      const response = await fetch("http://localhost:5000/send_message", {
        method: "POST",
        headers: { "Content-Type": "application/json" },
        body: JSON.stringify(request),
      });

      if (!response.ok) throw new Error("Failed to send message");
      const data = await response.json();
      updateMessages(activeTab, {
        user: "ROS",
        message: data.response,
        type: "default",
      });
    } catch (error) {
      console.error("Error:", error.message);
      updateMessages(activeTab, {
        user: "System",
        message: "Error: Unable to reach the backend server.",
        type: "alert",
      });
    }
  }

  const handleSend = async () => {
    if (!input.trim()) return;

    updateMessages(activeTab, { user: "You", message: input.trim(), type: "default" });

    const namespace = tabs.find((tab) => tab.name === activeTab)?.namespace || null;
    const request = { message: input.trim(), namespace };
    setInput("");
    await sendMessageToBackend(request);
  };

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

  const handleKeyDown = (e) => {
    if (e.key === "Enter") {
      e.preventDefault();
      handleSend();
    }
  };

  const handleVoicePrompt = () => alert("Voice Prompt Activated! (🎙️)");

  // ----------------- Socket Listener Setup -----------------

  const setupSocketListener = useCallback(() => {
    const handleRosMessage = (data) => {
      updateMessages("Global", {
        user: data.user,
        message: data.message,
        type: data.type || "default",
      });
    };

    socket.on("ros_message", handleRosMessage);

    // Cleanup function to remove the listener
    return () => {
      socket.off("ros_message", handleRosMessage);
    };
  }, []); // No dependencies because updateMessages is stable

  // ----------------- useEffect Hooks -----------------

  // Check server start time on mount
  useEffect(() => {
    fetch("http://localhost:5000/server_start_time")
      .then((response) => response.json())
      .then((data) => {
        const serverStartTime = data.server_start_time;
        console.log("Received server start time:", serverStartTime);
        const storedServerStartTime = localStorage.getItem(SERVER_START_TIME_KEY);
        console.log("Stored server start time:", storedServerStartTime);

        if (storedServerStartTime !== serverStartTime) {
          console.log("Server start time mismatch. Clearing local storage.");
          localStorage.clear();
          localStorage.setItem(SERVER_START_TIME_KEY, serverStartTime);

          // Reset state
          setTabs([{ name: "Global", namespace: null }]);
          setMessages({ Global: [] });
          setActiveTab("Global");
        } else {
          console.log("Server start time matches. No action needed.");
        }
      })
      .catch((error) => {
        console.error("Error fetching server start time:", error);
      });
  }, []);

  // Save tabs and messages to local storage
  useEffect(() => saveToStorage("chatTabs", tabs), [tabs]);
  useEffect(() => saveToStorage("chatMessages", messages), [messages]);

  // Setup Socket Listener
  useEffect(() => {
    const cleanup = setupSocketListener();
    return cleanup;
  }, [setupSocketListener]);

  // Auto-scroll to bottom when messages or activeTab change
  useEffect(() => {
    autoScrollToBottom();
  }, [messages, activeTab]);

  // -------------------- Render ------------------------
  return (
    <div className="chat-sub-panel">
      <div className="chat-container">
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

        <div className="chat-log" ref={chatLogRef}>
          {(messages[activeTab] || []).map((chat, idx) => (
            <p key={idx} className={`chat-message ${chat.type || ""}`}>
              <strong>{chat.user}:</strong> {chat.message}
            </p>
          ))}
        </div>

        <div className="chat-input-container">
          <input
            type="text"
            value={input}
            onChange={(e) => setInput(e.target.value)}
            onKeyDown={handleKeyDown}
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
