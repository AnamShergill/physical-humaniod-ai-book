"""
Reusable agent skill for lesson generation
"""
from typing import Dict, Any, List
import openai
from jinja2 import Template


class LessonGenerator:
    """
    Agent skill for generating lessons based on curriculum requirements
    """
    def __init__(self, llm_client=None):
        self.llm_client = llm_client  # Could be OpenAI, Anthropic, etc.

    def generate_lesson(
        self,
        topic: str,
        objectives: List[str],
        target_audience: str = "intermediate",
        lesson_length: str = "medium"  # short, medium, long
    ) -> Dict[str, Any]:
        """
        Generate a complete lesson based on the provided parameters
        """
        # Define lesson structure template
        lesson_template = Template("""
# Lesson: {{ topic }}

## Overview
{{ overview }}

## Learning Objectives
{% for objective in objectives %}
- {{ objective }}
{% endfor %}

## Content
{{ content }}

## Practical Exercise
{{ practical_exercise }}

## Summary
{{ summary }}

## Next Steps
{{ next_steps }}
        """)

        # Generate content sections using LLM
        prompt = self._create_generation_prompt(topic, objectives, target_audience, lesson_length)

        # In a real implementation, this would call the LLM
        # For now, we'll create a mock response
        lesson_content = self._mock_generate_content(topic, objectives, target_audience, lesson_length)

        # Render the lesson using the template
        lesson_text = lesson_template.render(
            topic=topic,
            overview=lesson_content['overview'],
            objectives=objectives,
            content=lesson_content['content'],
            practical_exercise=lesson_content['practical_exercise'],
            summary=lesson_content['summary'],
            next_steps=lesson_content['next_steps']
        )

        return {
            "topic": topic,
            "objectives": objectives,
            "content": lesson_text,
            "metadata": {
                "target_audience": target_audience,
                "length": lesson_length,
                "generated_at": "2024-01-01T00:00:00Z"
            }
        }

    def _create_generation_prompt(self, topic: str, objectives: List[str], target_audience: str, lesson_length: str) -> str:
        """
        Create a detailed prompt for lesson generation
        """
        length_guidelines = {
            "short": "100-300 words, 2-3 main sections",
            "medium": "300-600 words, 3-5 main sections with examples",
            "long": "600+ words, 5+ sections with detailed explanations and exercises"
        }

        return f"""
Create an educational lesson about "{topic}" for {target_audience} learners.
Learning objectives: {', '.join(objectives)}
Lesson length: {length_guidelines[lesson_length]}

The lesson should follow this structure:
1. Overview - Brief introduction to the topic
2. Learning Objectives - The provided list
3. Content - Main educational content with examples where appropriate
4. Practical Exercise - Hands-on activity or thought exercise
5. Summary - Key takeaways
6. Next Steps - What to learn next

Use clear, educational language appropriate for the target audience.
Include practical examples and applications where relevant.
Format in Markdown.
        """

    def _mock_generate_content(self, topic: str, objectives: List[str], target_audience: str, lesson_length: str) -> Dict[str, str]:
        """
        Mock content generation for demonstration
        """
        return {
            "overview": f"This lesson covers {topic}, an important concept in robotics and AI.",
            "content": f"Content about {topic} would go here with detailed explanations and examples.",
            "practical_exercise": f"Try implementing a simple example of {topic} in your development environment.",
            "summary": f"In summary, {topic} is crucial for understanding advanced robotics concepts.",
            "next_steps": f"Next, we'll explore how {topic} integrates with other robotics systems."
        }

    def refine_lesson(self, existing_lesson: str, feedback: str) -> str:
        """
        Refine an existing lesson based on feedback
        """
        prompt = f"""
Refine the following lesson based on this feedback:

LESSON:
{existing_lesson}

FEEDBACK:
{feedback}

Provide an improved version of the lesson that addresses the feedback while maintaining the educational value.
        """

        # In a real implementation, this would call the LLM
        # For now, return the original lesson with a note
        return f"<!-- Refinement based on feedback: {feedback} -->\n{existing_lesson}"


# Global lesson generator instance
lesson_generator = LessonGenerator()