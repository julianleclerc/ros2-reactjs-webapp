import React from "react";
import PlannedActionPanel from "../../components/plannedActions/PlannedActionPanel.jsx";
import "./ActionInterfacePage.css";

const ActionInterfacePage = () => {
    return (
        <div className="action-interface-page">
            <div className="planned-action-panel">
                <PlannedActionPanel />
            </div>
        </div>
    );
};

export default ActionInterfacePage;
