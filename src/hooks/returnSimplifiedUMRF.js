function computeExecutionQueue(graph) {
  const actions = graph.actions;
  const graphEntry = graph.graph_entry || [];

  // Build a mapping of instance_id -> action object
  const actionDict = {};
  actions.forEach((action) => {
    actionDict[action.instance_id] = action;
  });

  // Initialize inDegree map and childrenMap
  const inDegree = {};
  const childrenMap = {};

  // Set inDegree for each action to 0 initially.
  actions.forEach((action) => {
    inDegree[action.instance_id] = 0;
  });

  // Populate childrenMap and compute inDegree from each action's children.
  actions.forEach((action) => {
    if (action.children) {
      action.children.forEach((child) => {
        const childId = child.instance_id;
        inDegree[childId] = (inDegree[childId] || 0) + 1;
        if (!childrenMap[action.instance_id]) {
          childrenMap[action.instance_id] = [];
        }
        childrenMap[action.instance_id].push(childId);
      });
    }
  });

  // Assign level 0 to all graph entry actions.
  const levels = {};
  graphEntry.forEach((entry) => {
    levels[entry.instance_id] = 0;
  });

  // Also assign level 0 to any action with no parents (if not already assigned).
  actions.forEach((action) => {
    const id = action.instance_id;
    if (!(id in levels) && inDegree[id] === 0) {
      levels[id] = 0;
    }
  });

  // Process actions in topological order using a queue.
  const queue = Object.keys(levels).map(Number);
  while (queue.length > 0) {
    const aid = queue.shift();
    const currentLevel = levels[aid];
    const children = childrenMap[aid] || [];
    children.forEach((childId) => {
      const newLevel = currentLevel + 1;
      if (!(childId in levels) || newLevel > levels[childId]) {
        levels[childId] = newLevel;
      }
      inDegree[childId] -= 1;
      if (inDegree[childId] === 0) {
        queue.push(childId);
      }
    });
  }

  // Determine the maximum level.
  let maxLevel = 0;
  Object.keys(levels).forEach((key) => {
    if (levels[key] > maxLevel) {
      maxLevel = levels[key];
    }
  });

  // Group actions by level into the final queue.
  const queueSteps = [];
  for (let level = 0; level <= maxLevel; level++) {
    const step = {};
    Object.keys(levels).forEach((key) => {
      if (levels[key] === level) {
        const action = actionDict[key];
        // Extract parameters, ensuring all pvf_values are strings
        const params = {};
        if (action.input_parameters) {
          Object.keys(action.input_parameters).forEach((paramKey) => {
            params[paramKey] = String(action.input_parameters[paramKey].pvf_value);
          });
        }
        step[action.name] = params;
      }
    });
    if (Object.keys(step).length > 0) {
      queueSteps.push(step);
    }
  }

  return { queue: queueSteps };
}

// Example usage:
const inputGraph = {
  "graph_name": "Go straight then stop once you see a chair and finally examine what you see.",
  "graph_description": "inspecting the chair",
  "graph_entry": [
    {
      "name": "move",
      "instance_id": 0
    },
    {
      "name": "check",
      "instance_id": 1
    }
  ],
  "graph_exit": [
    {
      "name": "examine",
      "instance_id": 4
    }
  ],
  "actions": [
    {
      "name": "move",
      "instance_id": 0,
      "type": "sync",
      "input_parameters": {
        "orientation": {"pvf_type": "string", "pvf_value": "straight"}
      },
      "children": [
        {
          "name": "stop",
          "instance_id": 2
        }
      ]
    },
    {
      "name": "check",
      "instance_id": 1,
      "type": "sync",
      "input_parameters": {
        "object": {"pvf_type": "string", "pvf_value": "chair"}
      },
      "children": [
        {
          "name": "stop",
          "instance_id": 2
        }
      ]
    },
    {
      "name": "stop",
      "instance_id": 2,
      "type": "sync",
      "input_parameters": {
        "time": {"pvf_type": "string", "pvf_value": 1}
      },
      "children": [
        {
          "name": "examine",
          "instance_id": 3
        }
      ],
      "parents": [
        {
          "name": "move",
          "instance_id": 0
        },
        {
          "name": "check",
          "instance_id": 1
        }
      ]
    },
    {
      "name": "examine",
      "instance_id": 3,
      "type": "sync",
      "input_parameters": {
        "object": {"pvf_type": "string", "pvf_value": "chair"}
      },
      "parents": [
        {
          "name": "stop",
          "instance_id": 2
        }
      ]
    }
  ]
};

const result = computeExecutionQueue(inputGraph);
console.log(JSON.stringify(result, null, 4));
