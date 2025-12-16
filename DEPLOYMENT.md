# Physical AI & Humanoid Robotics Textbook

An AI-native textbook covering Physical AI & Humanoid Robotics, optimized for fast loading, mobile-friendliness, and efficient learning.

## Features

- **Fast Loading**: Optimized for < 45 minutes total reading time
- **Mobile-Friendly**: Responsive design for all devices
- **Scannable Content**: Focused lessons with clear structure
- **RAG System**: Lesson-level granularity for accurate answers
- **Personalization**: Adapts to learning preferences
- **Multilingual**: High-quality Urdu translation support

## Architecture

### Frontend
- **Platform**: Docusaurus
- **Deployment**: Vercel
- **Performance**: Optimized CSS and mobile-first design

### Backend
- **Platform**: Node.js/Express
- **Deployment**: Railway
- **API**: RESTful endpoints for content management

### Databases
- **Vector Database**: Qdrant (for RAG system)
- **Relational Database**: Neon PostgreSQL (for user data and content management)

## Deployment

### Frontend (Vercel)
1. Connect your GitHub repository to Vercel
2. Set build command: `npm run build`
3. Set output directory: `build`
4. Configure environment variables:
   - `ALGOLIA_APP_ID` (optional)
   - `ALGOLIA_API_KEY` (optional)
   - `ALGOLIA_INDEX_NAME` (optional)

### Backend (Railway)
1. Import the `railway.json` configuration
2. Set environment variables:
   - `NODE_ENV=production`
   - `PORT=8080`
   - `DATABASE_URL` (Neon connection string)
   - `QDRANT_URL` (Qdrant connection string)
   - `NEON_DATABASE_URL` (Neon connection string)

### Vector Database (Qdrant)
1. Deploy Qdrant instance
2. Configure with `qdrant_config.yaml`
3. Set up collections for lesson embeddings
4. Connect to backend API

### Relational Database (Neon)
1. Create Neon PostgreSQL project
2. Apply database schema from `neon_config.yaml`
3. Enable Row Level Security
4. Set up connection pooling

## Content Structure

The textbook is organized into chapters with scannable lessons:

- Chapter 1: Foundations of Physical AI
- Chapter 2: Overview of Humanoid Robotics
- Chapter 3: ROS 2 Architecture
- Chapter 4: Python Integration & URDF
- Chapter 5: Gazebo Physics Simulation
- Chapter 6: Unity Visualization & Interaction
- Chapter 7: Isaac Sim SDK Basics
- Chapter 8: Isaac ROS Navigation
- Chapter 9: Hardware & Lab Setup

## Performance Optimizations

- Mobile-first responsive design
- Optimized images and assets
- Efficient code block rendering
- Fast loading animations
- Accessibility improvements
- Dark mode support
- Reduced motion options

## RAG System

The textbook includes a Retrieval Augmented Generation system that provides:

- Lesson-level granularity for accurate citations
- Fast vector search using Qdrant
- Semantic similarity matching
- Context-aware responses

## Personalization

- User progress tracking
- Reading preferences
- Language selection
- Adaptive content delivery

## Technologies Used

- **Frontend**: Docusaurus, React, CSS
- **Backend**: Node.js, Express
- **Databases**: PostgreSQL (Neon), Vector DB (Qdrant)
- **Deployment**: Vercel, Railway
- **CI/CD**: GitHub Actions
- **Monitoring**: Built-in logging and metrics