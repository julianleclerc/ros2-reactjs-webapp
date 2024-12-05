import React, { useState, useEffect } from "react";
import "./App.css";
import MenuPanel from "../components/menu/MenuPanel";
import ChatInterfacePage from "../pages/chatInterface/ChatInterfacePage";
import ActionInterfacePage from "../pages/actionInterface/ActionInterfacePage";
import useWindowDimensions from "../hooks/useWindowDimensions";

const App = () => {
  const [currentPage, setCurrentPage] = useState("ChatInterfacePage");
  const { aspectRatio, height } = useWindowDimensions();

  /* Thresholds to hide navigation bar and switch to chat interface */
  const aspectRatioThreshold = 0.6; // 0-1
  const heightThreshold = 500; // in px

  /* Automatically switch to chat interface if in mobile mode */
  const isMobile = aspectRatio > aspectRatioThreshold || height < heightThreshold;

  useEffect(() => {
    if (isMobile && currentPage !== "ChatInterfacePage") {
      setCurrentPage("ChatInterfacePage");
    }
  }, [isMobile, currentPage]);

  /* Setup page rendered */
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

  /* Render toolbar + page */
  return (
    <div className="webapp">
      {!isMobile && (
        <div className="nav">
          <MenuPanel setPage={setCurrentPage} />
        </div>
      )}
      <div className="app">{renderPage()}</div>
    </div>
  );
};

export default App;
