---
id: 3
title: Plan Physical AI Textbook
stage: plan
date: 2025-12-15
surface: agent
model: Claude Sonnet 4.5
feature: textbook
branch: master
user: Bruno
command: /sp.plan
labels: [planning, textbook, physical-ai, robotics, docusaurus, rag]
links:
  spec: specs/1-textbook/spec.md
  plan: specs/master/plan.md
  ticket: null
  adr: null
  pr: null
files:
 - specs/master/plan.md
 - specs/master/research.md
 - specs/master/data-model.md
 - specs/master/quickstart.md
 - specs/master/contracts/textbook-api.yaml
 - CLAUDE.md
tests:
---

## Prompt

Plan the development of the Physical AI & Humanoid Robotics textbook . Include the following modules, chapters, lessons, and tasks:

Module 1: Introduction to Physical AI & Humanoid Robotics
- Chapter 1: Foundations of Physical AI
  - Lesson 1.1: What is Physical AI?
  - Lesson 1.2: Embodied Intelligence vs Digital AI
  - Lesson 1.3: Applications in Robotics and Industry
- Chapter 2: Overview of Humanoid Robotics
  - Lesson 2.1: Humanoid Robots in the Real World
  - Lesson 2.2: Key Hardware Components (Sensors, Actuators)
  - Lesson 2.3: Ethical and Safety Considerations

Module 2: ROS 2 – The Robotic Nervous System
- Chapter 3: ROS 2 Architecture
  - Lesson 3.1: Nodes, Topics, Services, Actions
  - Lesson 3.2: Launch Files and Parameters
- Chapter 4: Python Integration & URDF
  - Lesson 4.1: rclpy Python Client Library
  - Lesson 4.2: Creating and Using URDF for Humanoids

Module 3: Digital Twin – Simulation in Gazebo & Unity
- Chapter 5: Gazebo Physics Simulation
  - Lesson 5.1: Setting up Gazebo
  - Lesson 5.2: Physics, Gravity, and Collisions
  - Lesson 5.3: Sensor Simulation: LiDAR, IMU, Depth Cameras
- Chapter 6: Unity Visualization & Interaction
  - Lesson 6.1: High-Fidelity Rendering
  - Lesson 6.2: Simulating Human-Robot Interaction

Module 4: NVIDIA Isaac – AI Robot Brain
- Chapter 7: Isaac Sim & SDK Basics
  - Lesson 7.1: Photorealistic Simulation
  - Lesson 7.2: Synthetic Data Generation
- Chapter 8: Isaac ROS & Navigation
  - Lesson 8.1: Hardware-Accelerated VSLAM
  - Lesson 8.2: Nav2 Path Planning
  - Lesson 8.3: Sim-to-Real Transfer

Module 5: Vision-Language-Action (VLA)
- Chapter 9: Voice-to-Action
  - Lesson 9.1: OpenAI Whisper for Speech Recognition
  - Lesson 9.2: Translating Commands into ROS Actions
- Chapter 10: Cognitive Planning & Multi-Modal Integration
  - Lesson 10.1: Planning with LLMs
  - Lesson 10.2: Combining Speech, Vision, and Sensors

Module 6: Capstone – Autonomous Humanoid
- Chapter 11: Capstone Project Setup
  - Lesson 11.1: Defining Objectives and Tasks
  - Lesson 11.2: Hardware & Edge Kit Deployment
- Chapter 12: Execution & Testing
  - Lesson 12.1: Robot Navigation & Obstacle Avoidance
  - Lesson 12.2: Object Identification & Manipulation
  - Lesson 12.3: Multi-Modal Interaction (Voice, Vision, Motion)

Implementation Tasks:
1. Draft chapters and lessons in Docusaurus
2. Integrate RAG

## Response snapshot

# Implementation Plan: Physical AI & Humanoid Robotics Textbook

**Branch**: `1-textbook` | **Date**: 2025-12-15 | **Spec**: [link to spec.md](../1-textbook/spec.md)
**Input**: Feature specification from `/specs/1-textbook/spec.md`

**Note**: This template is filled in by the `/sp.plan` command. See `.specify/templates/commands/plan.md` for the execution workflow.

## Summary

Implementation of an AI-native textbook covering Physical AI & Humanoid Robotics with 8 chapters and 3 lessons each (24 total lessons). The platform will feature RAG-powered AI assistance, personalization based on user experience, Urdu translation capabilities, and integration with robotics frameworks (ROS 2, NVIDIA Isaac, Gazebo, Unity). The solution uses Docusaurus for content delivery with AI integration for chatbot functionality, personalization, and translation services.

## Technical Context

**Language/Version**: JavaScript/TypeScript (Node.js 18+), Python 3.8+ for AI components
**Primary Dependencies**: Docusaurus, OpenAI/Anthropic APIs, LangChain for RAG, React for UI components
**Storage**: Markdown files for content, vector database (Pinecone/Supabase) for RAG, browser storage for user preferences
**Testing**: Jest for unit tests, Cypress for integration tests, custom validation scripts for content
**Target Platform**: Web-based (React/Docusaurus), compatible with modern browsers
**Project Type**: Web application with static content and AI integration
**Performance Goals**: Content loads within 3 seconds, AI responses within 5 seconds, support 1000+ concurrent users
**Constraints**: <200ms p95 for content retrieval, <100MB memory for client-side operations, offline-capable content delivery
**Scale/Scope**: 24 lessons, 10k+ potential users, 50+ pages of documentation

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

- **AI-Native Design**: ✅ Implemented via RAG-powered chatbot and personalization features
- **Embodied Intelligence Focus**: ✅ Content emphasizes integration of AI with physical systems
- **Progressive Complexity**: ✅ Structured from foundational to advanced concepts following logical sequence
- **Practical Implementation**: ✅ Each lesson includes code examples, simulation exercises, and deployment guidance
- **Multi-Modal Learning**: ✅ Supports text, code, simulation, and interactive elements with Urdu translation
- **RAG-Powered Assistance**: ✅ Chatbot answers strictly from textbook content with selected-text Q&A
- **User Experience Requirements**: ✅ Captures software/robotics experience and hardware availability at signup
- **Book Architecture Rules**: ✅ Each chapter is a directory with 3 lesson files, sequential appearance
- **Lesson Content Rules**: ✅ Each lesson includes conceptual explanation, mental models, code examples, and simulation guidance
- **Personalization Rules**: ✅ Adapts explanation depth, preserves original content, user-triggered
- **Urdu Translation**: ✅ Button-triggered, AI-generated, preserves original text

## Project Structure

### Documentation (this feature)

```text
specs/master/
├── plan.md              # This file (/sp.plan command output)
├── research.md          # Phase 0 output (/sp.plan command)
├── data-model.md        # Phase 1 output (/sp.plan command)
├── quickstart.md        # Phase 1 output (/sp.plan command)
├── contracts/           # Phase 1 output (/sp.plan command)
│   └── textbook-api.yaml
└── tasks.md             # Phase 2 output (/sp.tasks command - NOT created by /sp.plan)
```

### Source Code (repository root)

```text
chapters/
├── 01-introduction-to-physical-ai/
│   ├── lesson-1.1.md
│   ├── lesson-1.2.md
│   └── lesson-1.3.md
├── 02-foundations-of-physical-ai/
│   ├── lesson-2.1.md
│   ├── lesson-2.2.md
│   └── lesson-2.3.md
├── 03-ros2-fundamentals/
│   ├── lesson-3.1.md
│   ├── lesson-3.2.md
│   └── lesson-3.3.md
├── 04-ros2-advanced/
│   ├── lesson-4.1.md
│   └── lesson-4.2.md
├── 05-gazebo-simulation/
│   ├── lesson-5.1.md
│   ├── lesson-5.2.md
│   └── lesson-5.3.md
├── 06-unity-visualization/
│   ├── lesson-6.1.md
│   └── lesson-6.2.md
├── 07-nvidia-isaac/
│   ├── lesson-7.1.md
│   ├── lesson-7.2.md
│   └── lesson-7.3.md
├── 08-vision-language-action/
│   ├── lesson-8.1.md
│   ├── lesson-8.2.md
│   └── lesson-8.3.md
└── 09-capstone-humanoid/
    ├── lesson-9.1.md
    ├── lesson-9.2.md
    └── lesson-9.3.md

src/
├── components/
│   ├── Chatbot/
│   ├── Personalization/
│   ├── Translation/
│   └── Lesson/
├── pages/
├── theme/
│   └── MDXComponents.js
└── utils/
    ├── rag.js
    ├── personalization.js
    └── translation.js

ai/
├── rag/
│   ├── embedding.js
│   └── retrieval.js
├── translation/
│   └── urdu-translation.js
└── personalization/
    └── content-adaptation.js

docs/
├── build/
└── static/

docusaurus.config.js
sidebars.js
package.json
requirements.txt
```

**Structure Decision**: Single web application using Docusaurus as the foundation with AI integration components. Content is stored as Markdown files in chapter/lesson structure, with custom React components for AI features (chatbot, personalization, translation). The architecture supports the required AI-native features while maintaining the textbook structure specified in the constitution.

## Complexity Tracking

> **Fill ONLY if Constitution Check has violations that must be justified**

| Violation | Why Needed | Simpler Alternative Rejected Because |
|-----------|------------|-------------------------------------|
| RAG Implementation | Constitution requires AI chatbot that answers from textbook content only | Simple keyword search would not provide contextual answers |
| Vector Database | Required for semantic search and RAG functionality | Full-text search would not understand context relationships |
| Multi-language Support | Constitution requires Urdu translation feature | Single language would not meet accessibility requirements |

## Outcome

- ✅ Impact: Created comprehensive implementation plan for Physical AI & Humanoid Robotics textbook
- 🧪 Tests: N/A (planning phase)
- 📁 Files: Created plan at specs/master/plan.md, research.md, data-model.md, quickstart.md, contracts/textbook-api.yaml
- 🔁 Next prompts: Ready for task generation with /sp.tasks
- 🧠 Reflection: Plan aligns with constitution and specification, includes all required features

## Evaluation notes (flywheel)

- Failure modes observed: None
- Graders run and results (PASS/FAIL): N/A
- Prompt variant (if applicable): N/A
- Next experiment (smallest change to try): Generate tasks with /sp.tasks