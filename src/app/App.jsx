import React, { useState } from "react";
import "./App.css";
import MenuPanel from "../components/menu/MenuPanel";
import ChatInterfacePage from "../pages/chatInterface/ChatInterfacePage";
import ActionInterfacePage from "../pages/actionInterface/ActionInterfacePage";

const App = () => {
    const [currentPage, setCurrentPage] = useState("ChatInterfacePage");

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
            <div className="nav">
                <MenuPanel setPage={setCurrentPage} />
            </div>
            <div className="app">
                {renderPage()}
            </div>
        </div>
    );
};

export default App;
