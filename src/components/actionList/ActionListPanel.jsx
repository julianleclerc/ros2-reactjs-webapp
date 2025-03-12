import React, { useState, useEffect, forwardRef, useImperativeHandle } from 'react';
import './ActionListPanel.css';
import { useDnD } from './DnDContext.jsx';

const ActionListPanel = forwardRef(({ actionsIn, onActionSelect, selectedGraph }, ref) => {
    const [actions, setActions] = useState([]);
    const [activeAction, setActiveAction] = useState();
    const [_, setType] = useDnD();

    useEffect(() => {
        if (actionsIn && Array.isArray(actionsIn)) {
            setActions(actionsIn);
        }
    }, [actionsIn]);

    useImperativeHandle(ref, () => ({
        clearActiveAction: () => {
            setActiveAction(null);
        }
    }));

    const handleActionSelectClick = (action) => {
        setActiveAction(action);
        onActionSelect(action.name);
    };

    const onDragStart = (event, nodeType, actionName) => {
        // Always allow dragging, but warn if no graph is selected
        setType(nodeType);
        event.dataTransfer.effectAllowed = 'move';
        event.dataTransfer.setData('actionName', actionName);
        
        // Optional: Add visual feedback if no graph is selected
        if (!selectedGraph) {
            console.warn('Please select a graph before dropping an action');
        }

        console.log('Dragging:', nodeType, ", with action name: ", actionName);
    };

    return (
        <div>
            <div className="buttons-column">
                <h2>Actions</h2>
                {actions.length > 0 ? (
                    actions.map((action, index) => (
                        <div className="buttons-row" key={action.id}>
                            <button
                                className={`text-button ${activeAction?.name === action.name ? 'active-action' : ''}`}
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
});

export default ActionListPanel;
