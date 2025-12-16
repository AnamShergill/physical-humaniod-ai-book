/**
 * Creating a sidebar enables you to:
 - create an ordered group of docs
 - render a sidebar for each doc of that group
 - provide next/previous navigation

 The sidebars can be generated from the filesystem, or explicitly defined here.

 Create as many sidebars as you want.
 */

module.exports = {
  textbookSidebar: [
    {
      type: 'doc',
      id: 'intro',
    },
    {
      type: 'category',
      label: 'Physical AI & Humanoid Robotics Textbook',
      items: [
        {
          type: 'category',
          label: 'Chapter 1: Foundations of Physical AI',
          items: [
            'chapters/foundations-of-physical-ai/lesson-1.1',
            'chapters/foundations-of-physical-ai/lesson-1.2',
            'chapters/foundations-of-physical-ai/lesson-1.3',
            'chapters/foundations-of-physical-ai/lesson-1.4'
          ],
        },
        {
          type: 'category',
          label: 'Chapter 2: Overview of Humanoid Robotics',
          items: [
            'chapters/overview-of-humanoid-robotics/lesson-2.1',
            'chapters/overview-of-humanoid-robotics/lesson-2.2',
            'chapters/overview-of-humanoid-robotics/lesson-2.3'
          ],
        },
        {
          type: 'category',
          label: 'Chapter 3: ROS 2 Architecture',
          items: [
            'chapters/ros2-architecture/lesson-3.1',
            'chapters/ros2-architecture/lesson-3.2',
            'chapters/ros2-architecture/lesson-3.3'
          ],
        },
        {
          type: 'category',
          label: 'Chapter 4: Python Integration & URDF',
          items: [
            'chapters/python-integration-urdf/lesson-4.1',
            'chapters/python-integration-urdf/lesson-4.2',
            'chapters/python-integration-urdf/lesson-4.3'
          ],
        },
        {
          type: 'category',
          label: 'Chapter 5: Gazebo Physics Simulation',
          items: [
            'chapters/gazebo-physics-simulation/lesson-5.1',
            'chapters/gazebo-physics-simulation/lesson-5.2',
            'chapters/gazebo-physics-simulation/lesson-5.3'
          ],
        },
        {
          type: 'category',
          label: 'Chapter 6: Unity Visualization & Interaction',
          items: [
            'chapters/unity-visualization-interaction/lesson-6.1',
            'chapters/unity-visualization-interaction/lesson-6.2',
            'chapters/unity-visualization-interaction/lesson-6.3'
          ],
        },
        {
          type: 'category',
          label: 'Chapter 7: Isaac Sim SDK Basics',
          items: [
            'chapters/isaac-sim-sdk-basics/lesson-7.1',
            'chapters/isaac-sim-sdk-basics/lesson-7.2',
            'chapters/isaac-sim-sdk-basics/lesson-7.3'
          ],
        },
        {
          type: 'category',
          label: 'Chapter 8: Isaac ROS Navigation',
          items: [
            'chapters/isaac-ros-navigation/lesson-8.1',
            'chapters/isaac-ros-navigation/lesson-8.2',
            'chapters/isaac-ros-navigation/lesson-8.3'
          ],
        },
        {
          type: 'category',
          label: 'Hardware & Lab Setup',
          items: [
            'chapters/hardware-setup/rtx-workstation',
            'chapters/hardware-setup/jetson-edge-kit',
            'chapters/hardware-setup/realsense-sensors',
            'chapters/hardware-setup/cloud-simulation'
          ],
        },
      ],
    },
  ],
};
