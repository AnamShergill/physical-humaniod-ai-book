"""
Reusable agent skill for content summarization
"""
from typing import Dict, Any, List
import re


class Summarizer:
    """
    Agent skill for summarizing educational content
    """
    def __init__(self):
        self.max_summary_length = 500  # characters

    def summarize_content(self, content: str, target_length: int = 200) -> str:
        """
        Generate a summary of the provided content
        """
        # Clean the content
        cleaned_content = self._clean_content(content)

        # Extract key sentences using a simple algorithm
        sentences = self._split_into_sentences(cleaned_content)
        summary_sentences = self._extract_key_sentences(sentences, target_length)

        return ' '.join(summary_sentences)

    def summarize_lesson(self, lesson_content: str, focus_areas: List[str] = None) -> Dict[str, str]:
        """
        Generate a structured summary of a lesson
        """
        # Clean the content
        cleaned_content = self._clean_content(lesson_content)

        # Extract different parts of the lesson
        overview = self._extract_section(cleaned_content, ['## Overview', '## Introduction'])
        objectives = self._extract_section(cleaned_content, ['## Learning Objectives', '## Objectives'])
        content = self._extract_section(cleaned_content, ['## Content', '## Main Content', '## Body'])
        summary_section = self._extract_section(cleaned_content, ['## Summary', '## Conclusion'])

        # Generate summaries for each section
        return {
            "overview": self.summarize_content(overview, 100) if overview else "",
            "objectives": self._summarize_objectives(objectives) if objectives else "",
            "key_points": self.summarize_content(content, 150) if content else "",
            "summary": summary_section if summary_section else self.summarize_content(content, 100) if content else "",
            "overall_summary": self.summarize_content(cleaned_content, 200)
        }

    def _clean_content(self, content: str) -> str:
        """
        Clean content by removing code blocks and other non-text elements
        """
        # Remove markdown code blocks
        content = re.sub(r'```.*?```', '', content, flags=re.DOTALL)
        # Remove inline code
        content = re.sub(r'`[^`]*`', '', content)
        # Remove extra whitespace
        content = re.sub(r'\s+', ' ', content)
        return content.strip()

    def _split_into_sentences(self, text: str) -> List[str]:
        """
        Split text into sentences
        """
        # Simple sentence splitting
        sentences = re.split(r'[.!?]+', text)
        # Filter out empty sentences and strip whitespace
        return [s.strip() for s in sentences if s.strip()]

    def _extract_key_sentences(self, sentences: List[str], target_length: int) -> List[str]:
        """
        Extract key sentences that fit within the target length
        """
        selected_sentences = []
        current_length = 0

        for sentence in sentences:
            # Add sentence if it doesn't exceed the target length
            if current_length + len(sentence) <= target_length:
                selected_sentences.append(sentence)
                current_length += len(sentence) + 1  # +1 for space
            else:
                break

        return selected_sentences

    def _extract_section(self, content: str, section_headers: List[str]) -> str:
        """
        Extract content under specific section headers
        """
        for header in section_headers:
            # Look for the header and extract content until the next header
            pattern = rf'{re.escape(header)}\s*(.*?)(?=\n## |\Z)'
            match = re.search(pattern, content, re.DOTALL)
            if match:
                return match.group(1).strip()
        return ""

    def _summarize_objectives(self, objectives_text: str) -> str:
        """
        Summarize learning objectives
        """
        # Extract bullet points or numbered lists
        objectives = re.findall(r'[-*]\s+(.*?)(?=\n[-*]|\n[^-*]|\Z)', objectives_text)
        if not objectives:
            # Try numbered list
            objectives = re.findall(r'\d+\.\s+(.*?)(?=\n\d+\.|\n[^\d]|\Z)', objectives_text)

        return ', '.join(objectives[:3]) if objectives else objectives_text[:100]  # Return first 3 objectives or first 100 chars

    def create_quiz_from_summary(self, summary: str) -> List[Dict[str, Any]]:
        """
        Create a simple quiz based on the summary
        """
        # This is a simple implementation - in reality, you'd want more sophisticated NLP
        questions = []

        # Look for key concepts in the summary and create simple questions
        sentences = self._split_into_sentences(summary)

        for i, sentence in enumerate(sentences[:3]):  # Create max 3 questions
            if len(sentence) > 20:  # Only create questions for substantial sentences
                questions.append({
                    "question": f"What is the main concept in this sentence: '{sentence[:50]}...'",
                    "options": ["Option A", "Option B", "Option C", "Option D"],
                    "correct_answer": "Option A",
                    "explanation": f"Based on the content: {sentence}"
                })

        return questions


# Global summarizer instance
summarizer = Summarizer()