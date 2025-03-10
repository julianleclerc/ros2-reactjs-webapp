import React, { useState, useEffect } from 'react';
import './ActionListPanel.css';
import { useDnD } from './DnDContext.jsx';

const ActionListPanel = ({ actionsIn, onActionSelect }) => {
    const [actions, setActions] = useState([]);
    const [activeAction, setActiveAction] = useState();
    const [_, setType] = useDnD();

    useEffect(() => {
        if (actionsIn && Array.isArray(actionsIn)) {
            setActions(actionsIn);
        }
    }, [actionsIn]);

    const handleActionSelectClick = (action) => {
        setActiveAction(action);
        onActionSelect(action.action_name);
    };

    const onDragStart = (event, nodeType, actionName) => {
        setType(nodeType);
        event.dataTransfer.effectAllowed = 'move';
        event.dataTransfer.setData('actionName', actionName);
        console.log('Dragging:', nodeType);
    };

    return (
        <div>
            <div className="buttons-column">
                <h2>Actions</h2>
                {actions.length > 0 ? (
                    actions.map((action, index) => (
                        <div className="buttons-row" key={action.id}>
                            <button
                                className={`text-button ${activeAction?.name === action.name ? 'active' : ''}`}
                                onClick={() => handleActionSelectClick(action)}
                                onDragStart={(event) => onDragStart(event, 'action', action.name)}
                                draggable
                            >
                                {action.name}
                            </button>
                        </div>
                    ))
                ) : (
                    <p>No actions available</p>
                )}

            </div>
        </div>
    );
};

export default ActionListPanel;
