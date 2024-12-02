import React, { useState, useEffect } from "react";
import "./App.css";
import MenuPanel from "../components/menu/MenuPanel";
import ChatInterfacePage from "../pages/chatInterface/ChatInterfacePage";
import ActionInterfacePage from "../pages/actionInterface/ActionInterfacePage";
import useWindowDimensions from "../hooks/useWindowDimensions";

const App = () => {
  const [currentPage, setCurrentPage] = useState("ChatInterfacePage");
  const { aspectRatio } = useWindowDimensions();

  /* adjust ratio threshold to hide navigation bar and switch to chat interface */
  const aspectRatioThreshold = 0.5;

  /* if mobile mode: switch to chat interface */
  useEffect(() => {
    if (aspectRatio > aspectRatioThreshold && currentPage !== "ChatInterfacePage") {
      setCurrentPage("ChatInterfacePage");
    }
  }, [aspectRatio, currentPage]);

  /* setup pages */
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


  /* render toolbar + page */
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
