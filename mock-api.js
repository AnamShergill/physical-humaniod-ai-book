// Mock API server to simulate RAG backend functionality
const express = require('express');
const cors = require('cors');
const app = express();

app.use(cors());
app.use(express.json());

// Mock data for responses
const mockResponses = {
  "what is physical ai": "Physical AI refers to the intersection of artificial intelligence and physical systems, particularly focusing on embodied intelligence where AI agents interact with the physical world through sensors and actuators.",
  "what are humanoid robots": "Humanoid robots are robots with human-like features and form, designed to interact with human environments and potentially work alongside humans.",
  "what is this textbook about": "This textbook covers Physical AI & Humanoid Robotics, teaching how to build AI systems that interact with the physical world, including topics like ROS2, Gazebo simulation, Isaac Sim, and real-world robotics applications.",
  "default": "I'm an AI assistant for this Physical AI & Humanoid Robotics textbook. I can answer questions about the content. Please ask about Physical AI, robotics, or related topics."
};

// Mock chatbot endpoint
app.post('/api/v1/chatbot/query', (req, res) => {
  console.log('Received query:', req.body.query);

  const query = req.body.query.toLowerCase();
  let response = mockResponses.default;

  // Simple keyword matching for mock responses
  for (const [keyword, answer] of Object.entries(mockResponses)) {
    if (keyword !== 'default' && query.includes(keyword)) {
      response = answer;
      break;
    }
  }

  // Add some mock sources
  const sources = [
    "Chapter 1: Foundations of Physical AI",
    "Lesson 1.2: Embodied Intelligence vs Digital AI",
    "Chapter 2: Overview of Humanoid Robotics"
  ];

  res.json({
    response: response,
    sources: sources,
    confidence: 0.85
  });
});

// Mock RAG search endpoint
app.post('/api/v1/rag/search', (req, res) => {
  res.json({
    query: req.body.query,
    results: [
      { content: "Sample content related to your query", relevance_score: 0.9 }
    ],
    count: 1
  });
});

// Health check endpoint
app.get('/health', (req, res) => {
  res.json({ status: 'healthy', service: 'mock-rag-api' });
});

const PORT = 8000;
app.listen(PORT, '0.0.0.0', () => {
  console.log(`Mock RAG API server running on http://localhost:${PORT}`);
  console.log('Available endpoints:');
  console.log('  POST /api/v1/chatbot/query - Chatbot query endpoint');
  console.log('  POST /api/v1/rag/search - RAG search endpoint');
  console.log('  GET /health - Health check');
});