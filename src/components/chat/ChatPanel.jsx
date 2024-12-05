import React, { useState, useEffect, useRef, useCallback } from "react";
import "./ChatPanel.css";
import io from "socket.io-client";

// Socket.IO configuration: Connects the front end to the backend server at the specified address
const socket = io("http://localhost:5000");

// Key to store the server start time in localStorage
// Used to ensure the session data remains valid between server restarts
const SERVER_START_TIME_KEY = "serverStartTime";

const ChatPanel = () => {
  // ----------------- State Management -----------------

  // State to manage the chat tabs
  const [tabs, setTabs] = useState(() =>
    loadFromStorage("chatTabs", [{ name: "Global", namespace: null }])
  );

  // State to track the currently active chat tab
  const [activeTab, setActiveTab] = useState("Global");

  // State to store chat messages for each tab
  const [messages, setMessages] = useState(() =>
    loadFromStorage("chatMessages", { Global: [] })
  );

  // State for the current message input
  const [input, setInput] = useState("");

  // Reference to the chat log DOM element for scrolling functionality
  const chatLogRef = useRef(null);

  // ----------------- Helper Functions -----------------

  /**
   * Loads data from localStorage.
   * @param {string} key - The key to retrieve from storage.
   * @param {any} defaultValue - The default value to return if the key doesn't exist.
   * @returns The stored data or the default value.
   */
  function loadFromStorage(key, defaultValue) {
    const savedData = localStorage.getItem(key);
    return savedData ? JSON.parse(savedData) : defaultValue;
  }

  /**
   * Saves data to localStorage.
   * @param {string} key - The key to store the data under.
   * @param {any} value - The value to store.
   */
  function saveToStorage(key, value) {
    localStorage.setItem(key, JSON.stringify(value));
  }

  /**
   * Automatically scrolls the chat log to the bottom.
   */
  function autoScrollToBottom() {
    if (chatLogRef.current) {
      chatLogRef.current.scrollTop = chatLogRef.current.scrollHeight;
    }
  }

  /**
   * Updates messages in a specific tab.
   * @param {string} tabName - The name of the tab to update.
   * @param {object} newMessage - The new message to add.
   */
  function updateMessages(tabName, newMessage) {
    setMessages((prevMessages) => ({
      ...prevMessages,
      [tabName]: [...(prevMessages[tabName] || []), newMessage],
    }));
  }

  /**
   * Sends a message to the backend server.
   * @param {object} request - The message request payload.
   */
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

  // ----------------- Event Handlers -----------------

  /**
   * Handles sending a message from the user.
   */
  const handleSend = async () => {
    if (!input.trim()) return;

    updateMessages(activeTab, { user: "You", message: input.trim(), type: "default" });

    const namespace = tabs.find((tab) => tab.name === activeTab)?.namespace || null;
    const request = { message: input.trim(), namespace };
    setInput("");
    await sendMessageToBackend(request);
  };

  /**
   * Handles adding a new chat tab with a namespace.
   */
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

  /**
   * Handles the Enter key for sending messages.
   * @param {object} e - The keyboard event.
   */
  const handleKeyDown = (e) => {
    if (e.key === "Enter") {
      e.preventDefault();
      handleSend();
    }
  };

  /**
   * Simulates a voice prompt action (placeholder).
   */
  const handleVoicePrompt = () => alert("Voice Prompt Activated! (🎙️)");

  // ----------------- Socket Listener Setup -----------------

  /**
   * Sets up a listener for receiving messages from ROS via Socket.IO.
   * @returns Cleanup function to remove the listener.
   */
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

  // Checks the server start time on mount to validate localStorage data
  useEffect(() => {
    fetch("http://localhost:5000/server_start_time")
      .then((response) => response.json())
      .then((data) => {
        const serverStartTime = data.server_start_time;
        const storedServerStartTime = localStorage.getItem(SERVER_START_TIME_KEY);

        if (storedServerStartTime !== serverStartTime) {
          localStorage.clear();
          localStorage.setItem(SERVER_START_TIME_KEY, serverStartTime);

          // Reset state
          setTabs([{ name: "Global", namespace: null }]);
          setMessages({ Global: [] });
          setActiveTab("Global");
        }
      })
      .catch((error) => {
        console.error("Error fetching server start time:", error);
      });
  }, []);

  // Saves tabs and messages to localStorage whenever they change
  useEffect(() => saveToStorage("chatTabs", tabs), [tabs]);
  useEffect(() => saveToStorage("chatMessages", messages), [messages]);

  // Sets up the Socket.IO listener
  useEffect(() => {
    const cleanup = setupSocketListener();
    return cleanup;
  }, [setupSocketListener]);

  // Automatically scrolls to the bottom of the chat log when messages or activeTab change
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
