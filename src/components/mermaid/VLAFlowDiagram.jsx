import React from 'react';

/**
 * Mermaid diagram component for VLA (Vision-Language-Action) workflows
 * This component renders Mermaid diagrams for documentation
 */
const VLAFlowDiagram = ({ diagramCode, title = "VLA Flow Diagram" }) => {
  return (
    <div className="mermaid-diagram">
      <h4>{title}</h4>
      <div className="mermaid">
        {diagramCode}
      </div>
    </div>
  );
};

// Example usage diagrams for VLA workflows
export const VLAFlowChart = () => {
  const vlaFlow = `graph TD
    A[Voice Input] --> B[Whisper API]
    B --> C[Natural Language Processing]
    C --> D{Command Type?}
    D -->|Navigation| E[Move to Location]
    D -->|Manipulation| F[Grasp Object]
    D -->|Other| G[Execute Action]
    E --> H[Isaac Sim Navigation]
    F --> I[Isaac Sim Manipulation]
    G --> J[Isaac Sim Action Execution]

    K[ROS 2 Nodes] -.-> B
    K -.-> E
    K -.-> F
    K -.-> G`;

  return <VLAFlowDiagram diagramCode={vlaFlow} title="Vision-Language-Action Flow" />;
};

export const CognitivePlanningDiagram = () => {
  const cognitiveFlow = `graph TD
    A[User Command] --> B[Language Understanding]
    B --> C[Goal Decomposition]
    C --> D[Constraint Analysis]
    D --> E[Plan Generation]
    E --> F[Plan Validation]
    F --> G[Step-by-Step Execution]
    G --> H{Task Complete?}
    H -->|No| I[Perception Update]
    I --> J[Plan Adjustment]
    J --> G
    H -->|Yes| K[Task Complete]

    L[Environmental Sensors] --> D
    L --> I
    M[Robot State] --> F
    M --> G`;

  return <VLAFlowDiagram diagramCode={cognitiveFlow} title="Cognitive Planning Flow" />;
};

export const MultiModalDiagram = () => {
  const multiModalFlow = `graph LR
    A[Human Gesture] --> B[Gesture Recognition]
    C[Camera Input] --> D[Object Detection]
    B --> E[Gesture-Object Association]
    D --> E
    E --> F[Intent Interpretation]
    F --> G[Action Planning]
    G --> H[Robot Execution]

    I[Voice Command] --> J[Natural Language Processing]
    J --> F`;

  return <VLAFlowDiagram diagramCode={multiModalFlow} title="Multi-Modal Interaction Flow" />;
};

export default VLAFlowDiagram;