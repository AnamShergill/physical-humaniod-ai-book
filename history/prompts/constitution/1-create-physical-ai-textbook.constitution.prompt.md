---
id: 1
title: Create Physical AI Textbook
stage: constitution
date: 2025-12-15
surface: agent
model: Claude Sonnet 4.5
feature: none
branch: master
user: Bruno
command: /sp.constitution
labels: [constitution, textbook, physical-ai, robotics]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
 - .specify/memory/constitution.md
tests:
---

## Prompt

# Project Constitution: AI-Native Textbook — Physical AI & Humanoid Robotics

## 1. Identity

project_name: Physical AI & Humanoid Robotics
project_type: AI-Native Technical Textbook
organization: Panaversity
initiative: Hackathon I — AI-Native Book Creation

---

## 2. Mission

The mission of this project is to design, author, and publish an AI-native textbook that teaches **Physical AI & Humanoid Robotics**, focusing on **embodied intelligence**, **robot control**, and **human–robot interaction**.

The textbook must bridge the ling at signup
- Chapter-level personalization
- Urdu translation support

---

## 5. Book Architecture Rules

The book MUST follow this structure:

- Each chapter is a directory
- Each lesson is a single Markdown file
- Each chapter has exactly 3 lessons
- Sidebar order defines learning flow
- Chapters must appear sequentially

Each chapter MUST begin with:
- Objectives section
- Personalize Content button
- Urdu Translation button

---

## 6. Chapter Structure (Authoritative)

### Chapter 1: Introduction to Physical AI
- Lesson 1.1: From Digital AI to Embodied Intelligence
- Lesson 1.2: The Humanoid Robotics Landscape
- Lesson 1.3: Sensors and the Physical World

### Chapter 2: Foundations of Physical AI Systems
- Lesson 2.1: Physics, Perception, and Action Loops
- Lesson 2.2: Embodiment and World Models
- Lesson 2.3: AI in Human-Centered Environments

### Chapter 3: The Robotic Nervous System (ROS 2)
- Lesson 3.1: ROS 2 Architecture and Middleware
- Lesson 3.2: Nodes, Topics, Services, and Actions
- Lesson 3.3: Python Agents with rclpy and URDF

### Chapter 4: Simulation and Digital Twins
- Lesson 4.1: Gazebo Physics and Environment Simulation
- Lesson 4.2: URDF and SDF Robot Descriptions
- Lesson 4.3: Unity for Visualization and HRI

### Chapter 5: The AI-Robot Brain (NVIDIA Isaac)
- Lesson 5.1: Isaac Sim and Synthetic Data Generation
- Lesson 5.2: Isaac ROS, VSLAM, and Navigation
- Lesson 5.3: Sim-to-Real Transfer Techniques

### Chapter 6: Humanoid Robot Engineering
- Lesson 6.1: Kinematics and Dynamics
- Lesson 6.2: Bipedal Locomotion and Balance Control
- Lesson 6.3: Manipulation and Grasping

### Chapter 7: Vision-Language-Action Systems
- Lesson 7.1: Speech Recognition with Whisper
- Lesson 7.2: Language-to-Action Planning with LLMs
- Lesson 7.3: Multimodal Human–Robot Interaction

### Chapter 8: Capstone — The Autonomous Humanoid
- Lesson 8.1: System Architecture and Integration
- Lesson 8.2: End-to-End Voice-Driven Autonomy
- Lesson 8.3: Evaluation, Deployment, and Demo

---

## 7. Lesson Content Rules

Each lesson MUST include:
- Conceptual explanation
- System or mental model
- Code or configuration examples
- Simulation or deployment guidance
- Content optimized for RAG indexing

---

## 8. RAG Chatbot Constitution

The embedded chatbot MUST:
- Answer strictly from book content
- Support selected-text Q&A
- Be chapter-aware
- Respect authentication state
- Be extensible with agent skills

---

## 9. Personalization Rules

At signup, users MUST be asked about:
- Software experience
- Robotics experience
- Available hardware

Chapter-level personalization MUST:
- Adapt explanation depth
- Preserve original content
- Be user-triggered

Urdu translation MUST:
- Be button-triggered
- Be AI-generated
- Not overwrite original text

---

## 10. Assessment Alignment

The book MUST support:
- ROS 2 package development
- Gazebo simulation projects
- Isaac perception pipelines
- Final humanoid capstone project

---

## 11. Deployment & Submission

The project MUST provide:
- Public GitHub repository
- Live deployed book
- Integrated chatbot
- Demo video under 90 seconds

---

## 12. Governance

This constitution is the single source of truth.

- All Claude agents MUST obey it
- All specs MUST align with it
- Manual edits are allowed but must preserve structure
- Any change requires constitution update

---

## 13. Success Definition

The project is successful if:
- It teaches Physical AI end-to-end
- It demonstrates embodied intelligence
- It functions as an AI-native book
- It meets Panaversity publication standards

## Response snapshot

<!-- SYNC IMPACT REPORT
Version change: N/A (new constitution) → v1.0.0
Modified principles: N/A
Added sections: All sections (new constitution)
Removed sections: N/A
Templates requiring updates:
- .specify/templates/plan-template.md ✅ updated
- .specify/templates/spec-template.md ✅ updated
- .specify/templates/tasks-template.md ✅ updated
- .specify/templates/commands/*.md ✅ reviewed
Follow-up TODOs: None
-->

# Project Constitution: AI-Native Textbook — Physical AI & Humanoid Robotics

## 1. Identity

**Project Name:** Physical AI & Humanoid Robotics
**Project Type:** AI-Native Technical Textbook
**Organization:** Panaversity
**Initiative:** Hackathon I — AI-Native Book Creation

---

## 2. Mission

The mission of this project is to design, author, and publish an AI-native textbook that teaches **Physical AI & Humanoid Robotics**, focusing on **embodied intelligence**, **robot control**, and **human–robot interaction**.

The textbook must bridge the gap between digital AI and physical robotics, enabling learners to understand and implement embodied intelligence systems.

---

## 3. Core Principles

### 3.1 AI-Native Design (Principle 1)
The textbook MUST be designed as an AI-native learning experience with embedded intelligence, personalized pathways, and interactive AI assistance. This means all content, exercises, and assessments are created to leverage AI capabilities for enhanced learning outcomes.

**Rationale:** Traditional textbooks cannot adequately serve the dynamic field of AI and robotics where concepts evolve rapidly and require interactive experimentation.

### 3.2 Embodied Intelligence Focus (Principle 2)
All content MUST emphasize the integration of AI algorithms with physical systems, covering sensors, actuators, perception-action loops, and real-world interaction challenges. Theory must be grounded in practical embodiment.

**Rationale:** Physical AI requires understanding how AI operates within physical constraints and environmental interactions, not just abstract computation.

### 3.3 Progressive Complexity (Principle 3)
Content MUST progress from foundational concepts to advanced implementations following a logical sequence that builds upon previous knowledge. Each chapter assumes only the knowledge from prior chapters.

**Rationale:** Learners need scaffolded learning experiences that gradually build competence without overwhelming cognitive load.

### 3.4 Practical Implementation (Principle 4)
Every concept MUST be accompanied by practical examples, code samples, simulation exercises, or deployment guidance that learners can execute and experiment with. Theory without practice is incomplete.

**Rationale:** Robotics is inherently hands-on; learners must be able to implement and test concepts to truly understand them.

### 3.5 Multi-Modal Learning (Principle 5)
The textbook MUST support diverse learning modalities including text, code, simulation, video, and interactive elements. Content MUST be accessible in multiple languages with Urdu as the primary alternative.

**Rationale:** Different learners have different preferences and needs; accessibility across languages ensures broader reach and inclusion.

### 3.6 RAG-Powered Assistance (Principle 6)
The embedded chatbot MUST provide contextual assistance based solely on the textbook content, supporting selected-text Q&A and chapter-aware responses. The system MUST be extensible with additional AI skills.

**Rationale:** Immediate, contextually relevant assistance enhances learning effectiveness and reduces friction in understanding complex concepts.

---

## 4. User Experience Requirements

At signup, users MUST be asked about:
- Software experience
- Robotics experience
- Available hardware

Chapter-level personalization MUST:
- Adapt explanation depth
- Preserve original content
- Be user-triggered

Urdu translation MUST:
- Be button-triggered
- Be AI-generated
- Not overwrite original text

---

## 5. Book Architecture Rules

The book MUST follow this structure:

- Each chapter is a directory
- Each lesson is a single Markdown file
- Each chapter has exactly 3 lessons
- Sidebar order defines learning flow
- Chapters must appear sequentially

Each chapter MUST begin with:
- Objectives section
- Personalize Content button
- Urdu Translation button

---

## 6. Chapter Structure (Authoritative)

### Chapter 1: Introduction to Physical AI
- Lesson 1.1: From Digital AI to Embodied Intelligence
- Lesson 1.2: The Humanoid Robotics Landscape
- Lesson 1.3: Sensors and the Physical World

### Chapter 2: Foundations of Physical AI Systems
- Lesson 2.1: Physics, Perception, and Action Loops
- Lesson 2.2: Embodiment and World Models
- Lesson 2.3: AI in Human-Centered Environments

### Chapter 3: The Robotic Nervous System (ROS 2)
- Lesson 3.1: ROS 2 Architecture and Middleware
- Lesson 3.2: Nodes, Topics, Services, and Actions
- Lesson 3.3: Python Agents with rclpy and URDF

### Chapter 4: Simulation and Digital Twins
- Lesson 4.1: Gazebo Physics and Environment Simulation
- Lesson 4.2: URDF and SDF Robot Descriptions
- Lesson 4.3: Unity for Visualization and HRI

### Chapter 5: The AI-Robot Brain (NVIDIA Isaac)
- Lesson 5.1: Isaac Sim and Synthetic Data Generation
- Lesson 5.2: Isaac ROS, VSLAM, and Navigation
- Lesson 5.3: Sim-to-Real Transfer Techniques

### Chapter 6: Humanoid Robot Engineering
- Lesson 6.1: Kinematics and Dynamics
- Lesson 6.2: Bipedal Locomotion and Balance Control
- Lesson 6.3: Manipulation and Grasping

### Chapter 7: Vision-Language-Action Systems
- Lesson 7.1: Speech Recognition with Whisper
- Lesson 7.2: Language-to-Action Planning with LLMs
- Lesson 7.3: Multimodal Human–Robot Interaction

### Chapter 8: Capstone — The Autonomous Humanoid
- Lesson 8.1: System Architecture and Integration
- Lesson 8.2: End-to-End Voice-Driven Autonomy
- Lesson 8.3: Evaluation, Deployment, and Demo

---

## 7. Lesson Content Rules

Each lesson MUST include:
- Conceptual explanation
- System or mental model
- Code or configuration examples
- Simulation or deployment guidance
- Content optimized for RAG indexing

---

## 8. RAG Chatbot Constitution

The embedded chatbot MUST:
- Answer strictly from book content
- Support selected-text Q&A
- Be chapter-aware
- Respect authentication state
- Be extensible with agent skills

---

## 9. Personalization Rules

At signup, users MUST be asked about:
- Software experience
- Robotics experience
- Available hardware

Chapter-level personalization MUST:
- Adapt explanation depth
- Preserve original content
- Be user-triggered

Urdu translation MUST:
- Be button-triggered
- Be AI-generated
- Not overwrite original text

---

## 10. Assessment Alignment

The book MUST support:
- ROS 2 package development
- Gazebo simulation projects
- Isaac perception pipelines
- Final humanoid capstone project

---

## 11. Deployment & Submission

The project MUST provide:
- Public GitHub repository
- Live deployed book
- Integrated chatbot
- Demo video under 90 seconds

---

## 12. Governance

This constitution is the single source of truth.

- All Claude agents MUST obey it
- All specs MUST align with it
- Manual edits are allowed but must preserve structure
- Any change requires constitution update

---

## 13. Success Definition

The project is successful if:
- It teaches Physical AI end-to-end
- It demonstrates embodied intelligence
- It functions as an AI-native book
- It meets Panaversity publication standards

---

## 14. Version Information

- **Constitution Version:** v1.0.0
- **Ratification Date:** 2025-12-15
- **Last Amended:** 2025-12-15
- **Amendment Procedure:** Changes require explicit approval and version increment following semantic versioning rules

---

## Outcome

- ✅ Impact: Created comprehensive project constitution for Physical AI & Humanoid Robotics textbook
- 🧪 Tests: N/A
- 📁 Files: Updated .specify/memory/constitution.md with complete constitution
- 🔁 Next prompts: Create feature specifications based on constitution
- 🧠 Reflection: Constitution captures all essential requirements for AI-native textbook project

## Evaluation notes (flywheel)

- Failure modes observed: None
- Graders run and results (PASS/FAIL): N/A
- Prompt variant (if applicable): N/A
- Next experiment (smallest change to try): Begin creating feature specifications based on constitution