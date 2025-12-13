import type { SidebarsConfig } from '@docusaurus/plugin-content-docs';

// This runs in Node.js - Don't use client-side code here (browser APIs, JSX...)

const sidebars: SidebarsConfig = {
  tutorialSidebar: [
    {
      type: 'category',
      label: 'Introduction',
      items: ['intro'],
    },

    {
      type: 'category',
      label: 'Module 1: The Robotic Nervous System (ROS 2)',
      items: [
        'modules/module-01-ros2-nervous-system/index',
        {
          type: 'category',
          label: 'Chapter 1: ROS 2 Architecture & Core Concepts',
          items: [
            'chapter-1-ros2-architecture/index',
            'chapter-1-ros2-architecture/concepts',
            'chapter-1-ros2-architecture/embodiment',
            'chapter-1-ros2-architecture/quiz',
          ],
        },
        {
          type: 'category',
          label: 'Chapter 2: Nodes, Topics, Services, Actions',
          items: [
            'chapter-2-nodes-topics-services/index',
            'chapter-2-nodes-topics-services/nodes',
            'chapter-2-nodes-topics-services/topics',
            'chapter-2-nodes-topics-services/services',
            'chapter-2-nodes-topics-services/actions',
            'chapter-2-nodes-topics-services/exercise1',
          ],
        },
        {
          type: 'category',
          label: 'Chapter 3: Building ROS 2 Packages with Python (rclpy)',
          items: [
            'chapter-3-python-packages/index',
            'chapter-3-python-packages/packages',
            'chapter-3-python-packages/launch-files',
            'chapter-3-python-packages/parameters',
            'chapter-3-python-packages/urdf',
            'chapter-3-python-packages/exercise2',
          ],
        },
        {
          type: 'category',
          label: 'Chapter 4: Launch Files, Parameters, and URDF for Humanoids',
          items: [
            'chapter-4-urdf-launch-files/index',
          ],
        },
      ],
    },
    {
      type: 'category',
      label: 'Module 2: The Digital Twin (Gazebo & Unity)',
      items: [
        'modules/module-02-digital-twin-sim/index',
        {
          type: 'category',
          label: 'Chapter 1: Gazebo Setup & Physics',
          items: [
            'modules/module-02-digital-twin-sim/chapter-1-gazebo-setup-physics/index',
          ],
        },
        {
          type: 'category',
          label: 'Chapter 2: Sensor Simulation',
          items: [
            'modules/module-02-digital-twin-sim/chapter-2-sensor-simulation/index',
          ],
        },
        {
          type: 'category',
          label: 'Chapter 3: Unity Integration & Rendering',
          items: [
            'modules/module-02-digital-twin-sim/chapter-3-unity-integration-rendering/index',
          ],
        },
      ],
    },
    {
      type: 'category',
      label: 'Module 3: The AI-Robot Brain (NVIDIA Isaac™)',
      items: [
        'modules/module-03-ai-robot-brain/index',
        {
          type: 'category',
          label: 'Chapter 1: Isaac SDK & Sim',
          items: [
            'modules/module-03-ai-robot-brain/chapter-1-isaac-sdk-sim/index',
          ],
        },
        {
          type: 'category',
          label: 'Chapter 2: Perception & Manipulation',
          items: [
            'modules/module-03-ai-robot-brain/chapter-2-perception-manipulation/index',
          ],
        },
        {
          type: 'category',
          label: 'Chapter 3: Reinforcement Learning Control',
          items: [
            'modules/module-03-ai-robot-brain/chapter-3-rl-control/index',
          ],
        },
        {
          type: 'category',
          label: 'Chapter 4: Sim-to-Real Transfer',
          items: [
            'modules/module-03-ai-robot-brain/chapter-4-sim-to-real/index',
          ],
        },
      ],
    },
    {
      type: 'category',
      label: 'Module 4: Vision-Language-Action (VLA)',
      items: [
        'modules/module-04-vla/README',
        'modules/module-04-vla/chapter-01-voice-speech-integration',
        'modules/module-04-vla/chapter-02-llm-planning',
        'modules/module-04-vla/chapter-03-multi-modal-hri',
        'modules/module-04-vla/quiz',
      ],
    },
  ],
};

export default sidebars;