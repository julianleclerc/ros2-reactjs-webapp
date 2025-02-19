import React, { useState, useEffect, useRef } from "react";
import "./ChatPanel.css";
import io from "socket.io-client";

// Establish connection to the backend server.
const socket = io("http://localhost:4000");

const ChatPanel = () => {
  // The full chat log is maintained from the backend.
  const [messages, setMessages] = useState({});
  // Instead of a single active tab, we now use an array for multiple selections.
  const [activeTabs, setActiveTabs] = useState([]);
  const [input, setInput] = useState("");

  // Ref for auto-scrolling the chat log.
  const chatLogRef = useRef(null);


  // on page refresh
  const refreshChatInterface = async () => {
    try {
      await fetch("http://localhost:4000/chat_interface_page_refresh", {
        method: "POST",
        headers: { "Content-Type": "application/json" },
      });
      console.log("Chat interface refreshed");
    } catch (error) {
      console.error("Error refreshing chat interface:", error);
    }
  };

  useEffect(() => {
    // Call refresh on mount.
    refreshChatInterface();
  
    // Define a handler that calls refreshChatInterface.
    const onPageShow = () => refreshChatInterface();
  
    // Listen for the 'pageshow' event (fires on load/reload)
    window.addEventListener("pageshow", onPageShow);
    // Listen for 'popstate' to capture back/forward navigation.
    window.addEventListener("popstate", onPageShow);
  
    return () => {
      window.removeEventListener("pageshow", onPageShow);
      window.removeEventListener("popstate", onPageShow);
    };
  }, []);

  useEffect(() => {
    socket.on("connect", () => {
      refreshChatInterface();
    });
    // Cleanup: remove the listener when the component unmounts.
    return () => {
      socket.off("connect");
    };
  }, []);
  
  /**
   * Scrolls the chat log to the bottom.
   */
  const autoScrollToBottom = () => {
    if (chatLogRef.current) {
      chatLogRef.current.scrollTop = chatLogRef.current.scrollHeight;
    }
  };

  /**
   * Listen for the "chat_log" event from the backend.
   * When received, update the local messages state.
   */
  useEffect(() => {
    socket.on("chat_log", (chatLog) => {
      setMessages(chatLog);
    });
    return () => {
      socket.off("chat_log");
    };
  }, []);

  /**
   * Auto-scroll when messages or active tabs change.
   */
  useEffect(() => {
    autoScrollToBottom();
  }, [messages, activeTabs]);

  /**
   * Sends a message to the backend.
   * The message will be sent to all selected actors.
   */
  const handleSend = async () => {
    if (!input.trim()) return;
    if (activeTabs.length === 0) {
      alert("Please select at least one actor to send the message.");
      return;
    }
    const request = {
      actor: activeTabs, // Send to all selected actors
      message: input.trim(),
    };

    setInput("");

    try {
      const response = await fetch("http://localhost:4000/send_message", {
        method: "POST",
        headers: { "Content-Type": "application/json" },
        body: JSON.stringify(request),
      });
      if (!response.ok) {
        console.error("Failed to send message");
      }
      // The backend will update and emit the updated chat_log.
    } catch (error) {
      console.error("Error sending message:", error);
    }
  };

  /**
   * Handles adding a new actor (tab).
   * This sends a POST request to the backend's /add_new_actor endpoint.
   */
  const handleAddTab = async () => {
    const actor = prompt("Enter the actor's name:");
    if (!actor) return;
    try {
      const response = await fetch("http://localhost:4000/add_new_actor", {
        method: "POST",
        headers: { "Content-Type": "application/json" },
        body: JSON.stringify({ actor_name: actor }),
      });
      const data = await response.json();
      if (!response.ok) {
        alert(data.error || "Error adding actor");
        return;
      }
      // Optionally auto-select the new actor.
      if (!activeTabs.includes(actor)) {
        setActiveTabs([...activeTabs, actor]);
      }
    } catch (error) {
      console.error("Error adding new actor:", error);
    }
  };

  /**
   * Toggles the selection of a tab (actor).
   */
  const handleToggleTab = (actor) => {
    if (activeTabs.includes(actor)) {
      setActiveTabs(activeTabs.filter((a) => a !== actor));
    } else {
      setActiveTabs([...activeTabs, actor]);
    }
  };

  /**
   * Handles the Enter key press to send messages.
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

  // Combine messages from all selected actors, remove duplicates, and sort them chronologically.
  let combinedMessages = [];
  if (activeTabs.length > 0) {
    activeTabs.forEach((actor) => {
      const actorMessages = messages[actor] || [];
      combinedMessages = combinedMessages.concat(actorMessages);
    });
    // Remove duplicates by creating a unique key for each message.
    const seen = new Set();
    combinedMessages = combinedMessages.filter((msg) => {
      const key = msg.join("|");
      if (seen.has(key)) return false;
      seen.add(key);
      return true;
    });
    // Sort by timestamp (assuming the first element is a valid timestamp).
    combinedMessages.sort((a, b) => new Date(a[0]) - new Date(b[0]));
  }

  

  return (
    <div className="chat-sub-panel">
      <div className="chat-container">
        {/* Tab Section */}
        <div className="tab-container">
          <button className="add-tab-button" onClick={handleAddTab}>
            +
          </button>
          <div className="tabs-scrollable">
            {Object.keys(messages).map((actor) => (
              <button
                key={actor}
                className={`tab-button ${activeTabs.includes(actor) ? "active-tab" : ""}`}
                onClick={() => handleToggleTab(actor)}
              >
                {actor}
              </button>
            ))}
          </div>
        </div>

        {/* Chat Log */}
        <div className="chat-log" ref={chatLogRef}>
          {activeTabs.length > 0 ? (
            combinedMessages.map((chat, idx) => {
              // Each chat entry is in the form [timestamp, user, message].
              const [timestamp, user, message] = chat;
              return (
                <p key={idx} className="chat-message">
                  <span className="chat-timestamp">{timestamp}</span>{" "}
                  <strong>{user}:</strong> {message}
                </p>
              );
            })
          ) : (
            <p>No actor selected. Please select at least one actor.</p>
          )}
        </div>

        {/* Input Section */}
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
