// src/components/chat/ChatPanel.jsx

import React, { useState, useEffect, useRef } from "react";
import "./ChatPanel.css";
import io from "socket.io-client";

const socket = io("http://localhost:5000");

const ChatPanel = () => {
  
  // ------------------- Session reset -------------------
  const resetSessionOnStart = false; 
  // -----------------------------------------------------

  // clears storage if reset true
  if (resetSessionOnStart) {
    sessionStorage.clear();
  }

  const [tabs, setTabs] = useState(() => {
    const savedTabs = sessionStorage.getItem("chatTabs");
    return savedTabs ? JSON.parse(savedTabs) : [{ name: "Global", namespace: null }];
  });

  const [activeTab, setActiveTab] = useState("Global");

  const [messages, setMessages] = useState(() => {
    const savedMessages = sessionStorage.getItem("chatMessages");
    return savedMessages ? JSON.parse(savedMessages) : { Global: [] };
  });

  const [input, setInput] = useState("");

  const chatLogRef = useRef(null);

  // Save tabs to sessionStorage whenever they change
  useEffect(() => {
    sessionStorage.setItem("chatTabs", JSON.stringify(tabs));
  }, [tabs]);

  // Save messages to sessionStorage whenever they change
  useEffect(() => {
    sessionStorage.setItem("chatMessages", JSON.stringify(messages));
  }, [messages]);

  // Set up Socket.IO listener once when the component mounts
  useEffect(() => {
    // When a 'ros_message' event is received, update the chat log
    socket.on("ros_message", (data) => {
      setMessages((prevMessages) => {
        const targetTab = "Global"; // Always append ROS messages to 'Global' tab

        const updatedMessages = {
          ...prevMessages,
          [targetTab]: [
            ...(prevMessages[targetTab] || []),
            {
              user: data.user,
              message: data.message,
              type: data.type || "default",
            },
          ],
        };

        return updatedMessages;
      });
    });

    // Clean up the event listener when the component unmounts
    return () => {
      socket.off("ros_message");
    };
  }, []);

  // Auto-scroll to bottom when messages change
  useEffect(() => {
    if (chatLogRef.current) {
      chatLogRef.current.scrollTop = chatLogRef.current.scrollHeight;
    }
  }, [messages, activeTab]);

  // Function to handle sending messages
  const handleSend = async () => {
    if (!input.trim()) return;

    // Add the user's message to the chat log
    setMessages((prevMessages) => {
      const updatedMessages = {
        ...prevMessages,
        [activeTab]: [
          ...(prevMessages[activeTab] || []),
          { user: "You", message: input.trim(), type: "default" },
        ],
      };

      return updatedMessages;
    });

    const request = {
      message: input.trim(),
      namespace: tabs.find((tab) => tab.name === activeTab)?.namespace || null,
    };
    setInput(""); // Clear the input field

    try {
      // Send the message to the backend
      const response = await fetch("http://localhost:5000/send_message", {
        method: "POST",
        headers: {
          "Content-Type": "application/json",
        },
        body: JSON.stringify(request),
      });

      if (response.ok) {
        const data = await response.json();
        // Add the backend response to the chat log
        setMessages((prevMessages) => {
          const updatedMessages = {
            ...prevMessages,
            [activeTab]: [
              ...(prevMessages[activeTab] || []),
              { user: "ROS", message: data.response, type: "default" },
            ],
          };

          return updatedMessages;
        });
      } else {
        console.error("Failed to send message to backend");
        // Optionally display an error message in the chat
        setMessages((prevMessages) => {
          const updatedMessages = {
            ...prevMessages,
            [activeTab]: [
              ...(prevMessages[activeTab] || []),
              {
                user: "System",
                message: "Error: Failed to communicate with the backend.",
                type: "alert",
              },
            ],
          };

          return updatedMessages;
        });
      }
    } catch (error) {
      console.error("Error communicating with backend:", error);
      // Optionally display an error message in the chat
      setMessages((prevMessages) => {
        const updatedMessages = {
          ...prevMessages,
          [activeTab]: [
            ...(prevMessages[activeTab] || []),
            {
              user: "System",
              message: "Error: Unable to reach the backend server.",
              type: "alert",
            },
          ],
        };

        return updatedMessages;
      });
    }
  };

  // Function to handle voice prompt activation (placeholder)
  const handleVoicePrompt = () => {
    alert("Voice Prompt Activated! (🎙️)"); // Implement voice input logic here
  };

  // Handle 'Enter' key press in the input field
  const handleKeyDown = (e) => {
    if (e.key === "Enter") {
      e.preventDefault();
      handleSend();
    }
  };

  // Function to add a new tab with a namespace
  const handleAddTab = () => {
    const namespace = prompt("Enter the robot namespace:");
    if (!namespace) return;

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
                className={`tab-button ${
                  activeTab === tab.name ? "active-tab" : ""
                }`}
                onClick={() => setActiveTab(tab.name)}
              >
                {tab.name}
              </button>
            ))}
          </div>
        </div>

        {/* Chat Log */}
        <div className="chat-log" ref={chatLogRef}>
          {(messages[activeTab] || []).map((chat, idx) => (
            <p key={idx} className={`chat-message ${chat.type || ""}`}>
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
