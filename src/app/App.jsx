// src/app/App.jsx

import React, { useState, useEffect } from "react";
import "./App.css";
import MenuPanel from "../components/menu/MenuPanel";
import ChatInterfacePage from "../pages/chatInterface/ChatInterfacePage";
import ActionInterfacePage from "../pages/actionInterface/ActionInterfacePage";
import useWindowDimensions from "../hooks/useWindowDimensions";

const App = () => {
  const [currentPage, setCurrentPage] = useState("ChatInterfacePage");
  const { aspectRatio } = useWindowDimensions();

  const aspectRatioThreshold = 0.5;

  // Effect to switch to ChatInterfacePage if in mobile layout
  useEffect(() => {
    if (aspectRatio > aspectRatioThreshold && currentPage !== "ChatInterfacePage") {
      setCurrentPage("ChatInterfacePage");
    }
  }, [aspectRatio, currentPage]);

  const renderPage = () => {
    switch (currentPage) {
      case "ChatInterfacePage":
        return <ChatInterfacePage />;
      case "ActionInterfacePage":
        return <ActionInterfacePage />;
      default:
        return <ChatInterfacePage />;
    }
  };

  return (
    <div style={{ display: "flex", minHeight: "100vh" }}>
      {aspectRatio <= aspectRatioThreshold && (
        <div className="nav">
          <MenuPanel setPage={setCurrentPage} />
        </div>
      )}
      <div className="app">
        {renderPage()}
      </div>
    </div>
  );
};

export default App;
