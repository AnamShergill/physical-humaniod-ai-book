# Quickstart Guide for Physical AI & Humanoid Robotics Textbook

## Overview
This guide will help you set up and start working with the Physical AI & Humanoid Robotics textbook project. The project is an AI-native educational platform that teaches embodied intelligence, robot control, and human-robot interaction.

## Prerequisites
- Node.js 18+ (for Docusaurus)
- Git
- Python 3.8+ (for AI/ML components)
- Access to NVIDIA Isaac platform (for simulation chapters)
- ROS 2 installation (for ROS 2 chapters)

## Setup Instructions

### 1. Clone the Repository
```bash
git clone [repository-url]
cd ai-textbook-physical-ai
```

### 2. Install Dependencies
```bash
# For Docusaurus
npm install

# For AI components
pip install -r requirements.txt
```

### 3. Environment Configuration
Create a `.env` file with the following:
```env
# AI/ML Configuration
OPENAI_API_KEY=your_openai_key_here
ANTHROPIC_API_KEY=your_anthropic_key_here

# Docusaurus Configuration
DOCUSAURUS_BASE_URL=/
DOCUSAURUS_PORT=3000

# RAG Configuration
RAG_EMBEDDING_MODEL=text-embedding-ada-002
RAG_SIMILARITY_THRESHOLD=0.7
```

### 4. Initialize the Textbook Content
```bash
# Build the Docusaurus site
npm run build

# Initialize the RAG database
npm run init-rag-db
```

## Project Structure
```
.
├── chapters/                 # Textbook content (8 chapters, 3 lessons each)
│   ├── 01-introduction-to-physical-ai/
│   │   ├── lesson-1.1.md
│   │   ├── lesson-1.2.md
│   │   └── lesson-1.3.md
│   ├── 02-foundations-of-physical-ai/
│   │   ├── lesson-2.1.md
│   │   ├── lesson-2.2.md
│   │   └── lesson-2.3.md
│   └── ... (additional chapters)
├── src/                     # Docusaurus custom components
│   ├── components/
│   ├── pages/
│   └── theme/
├── ai/                      # AI integration components
│   ├── rag/
│   ├── translation/
│   └── personalization/
├── docs/                    # Generated documentation
└── docusaurus.config.js     # Site configuration
```

## Running the Development Server
```bash
npm start
```
This will start the development server at `http://localhost:3000`.

## Adding New Content
1. Create a new lesson file in the appropriate chapter directory
2. Follow the lesson template format:
   - Title and objectives
   - Conceptual explanation
   - Code examples
   - Simulation exercises
   - Mental models/diagrams
3. Update the sidebar configuration in `sidebars.js`

## AI Features
### RAG Chatbot
The chatbot is integrated into each lesson page and can answer questions based on the textbook content. It uses vector embeddings to retrieve relevant content.

### Personalization
Users can personalize their learning experience based on their experience level. This adjusts the depth of explanations and complexity of examples.

### Urdu Translation
Each lesson has a toggle button to switch between English and Urdu content.

## Building for Production
```bash
npm run build
```
The output will be in the `build/` directory and can be served by any static hosting service.

## Testing
```bash
# Run content validation
npm run validate-content

# Run AI integration tests
npm run test-ai

# Run full test suite
npm run test
```

## Contributing
1. Create a feature branch
2. Add your content or improvements
3. Run tests
4. Submit a pull request

For more detailed information, refer to the full documentation in the `/docs` directory.