"""
Reusable agent skill for quiz generation
"""
from typing import Dict, Any, List
import re
import random


class QuizGenerator:
    """
    Agent skill for generating quizzes from lesson content
    """
    def __init__(self):
        self.question_types = ["multiple_choice", "true_false", "short_answer"]

    def generate_quiz(self, lesson_content: str, num_questions: int = 5, question_types: List[str] = None) -> Dict[str, Any]:
        """
        Generate a quiz based on lesson content
        """
        if question_types is None:
            question_types = ["multiple_choice"]  # Default to multiple choice

        # Extract key concepts from the lesson
        key_concepts = self._extract_key_concepts(lesson_content)

        # Generate questions based on key concepts
        questions = self._generate_questions_from_concepts(key_concepts, num_questions, question_types)

        return {
            "questions": questions,
            "total_questions": len(questions),
            "lesson_content_length": len(lesson_content),
            "difficulty": "intermediate",
            "generated_at": "2024-01-01T00:00:00Z"
        }

    def generate_mixed_quiz(self, lesson_content: str, config: Dict[str, int]) -> Dict[str, Any]:
        """
        Generate a mixed quiz with specified numbers of each question type
        """
        questions = []

        for q_type, count in config.items():
            if q_type == "multiple_choice":
                questions.extend(self._generate_multiple_choice_questions(lesson_content, count))
            elif q_type == "true_false":
                questions.extend(self._generate_true_false_questions(lesson_content, count))
            elif q_type == "short_answer":
                questions.extend(self._generate_short_answer_questions(lesson_content, count))

        # Shuffle the questions
        random.shuffle(questions)

        return {
            "questions": questions,
            "total_questions": len(questions),
            "breakdown": config,
            "generated_at": "2024-01-01T00:00:00Z"
        }

    def _extract_key_concepts(self, content: str) -> List[Dict[str, str]]:
        """
        Extract key concepts from lesson content
        """
        concepts = []

        # Remove code blocks and clean content
        cleaned_content = re.sub(r'```.*?```', '', content, flags=re.DOTALL)
        cleaned_content = re.sub(r'`[^`]*`', '', cleaned_content)

        # Extract headings as potential concepts
        headings = re.findall(r'#{2,3}\s*(.+)', cleaned_content)
        for heading in headings:
            concepts.append({
                "type": "heading",
                "text": heading.strip(),
                "context": self._find_context(cleaned_content, heading.strip())
            })

        # Extract bold text as important concepts
        bold_texts = re.findall(r'\*\*(.+?)\*\*', cleaned_content)
        for bold_text in bold_texts:
            if len(bold_text) > 3:  # Filter out short text
                concepts.append({
                    "type": "bold",
                    "text": bold_text.strip(),
                    "context": self._find_context(cleaned_content, bold_text.strip())
                })

        return concepts

    def _find_context(self, content: str, target: str) -> str:
        """
        Find the context around a target string
        """
        # Find the target in content and get surrounding text
        pattern = re.compile(re.escape(target), re.IGNORECASE)
        match = pattern.search(content)

        if match:
            start = max(0, match.start() - 100)
            end = min(len(content), match.end() + 100)
            return content[start:end].strip()

        return ""

    def _generate_questions_from_concepts(self, concepts: List[Dict[str, str]], num_questions: int, question_types: List[str]) -> List[Dict[str, Any]]:
        """
        Generate questions based on extracted concepts
        """
        questions = []

        for i in range(min(num_questions, len(concepts))):
            concept = concepts[i]
            q_type = random.choice(question_types)

            if q_type == "multiple_choice":
                question = self._create_multiple_choice_question(concept)
            elif q_type == "true_false":
                question = self._create_true_false_question(concept)
            elif q_type == "short_answer":
                question = self._create_short_answer_question(concept)
            else:
                question = self._create_multiple_choice_question(concept)  # Default

            if question:
                questions.append(question)

        return questions

    def _create_multiple_choice_question(self, concept: Dict[str, str]) -> Dict[str, Any]:
        """
        Create a multiple choice question based on a concept
        """
        # Create a question about the concept
        question_text = f"What is {concept['text']}?"

        # Generate wrong answers by slightly modifying the concept or using related terms
        wrong_answers = [
            f"An alternative to {concept['text']}",
            f"Something related to {concept['text']}",
            f"A different concept from {concept['text']}"
        ]

        # Shuffle wrong answers and take 3
        random.shuffle(wrong_answers)
        options = [concept['text']] + wrong_answers[:3]
        random.shuffle(options)

        # Find the correct answer index
        correct_index = options.index(concept['text'])
        correct_answer = f"Option {correct_index + 1}"

        return {
            "id": f"mc_{hash(concept['text']) % 10000}",
            "type": "multiple_choice",
            "question": question_text,
            "options": [f"Option {i+1}: {opt}" for i, opt in enumerate(options)],
            "correct_answer": correct_answer,
            "explanation": f"{concept['text']} is defined as: {concept['context'][:100]}...",
            "difficulty": "medium"
        }

    def _create_true_false_question(self, concept: Dict[str, str]) -> Dict[str, Any]:
        """
        Create a true/false question based on a concept
        """
        # Create a statement about the concept
        statement = f"{concept['text']} is an important concept in robotics."

        # Sometimes make the statement false to create false questions
        if random.choice([True, False]):
            question_text = f"True or False: {statement}"
            correct_answer = "True"
        else:
            # Create a false statement
            false_statement = f"{concept['text']} is not relevant to robotics."
            question_text = f"True or False: {false_statement}"
            correct_answer = "False"

        return {
            "id": f"tf_{hash(concept['text']) % 10000}",
            "type": "true_false",
            "question": question_text,
            "options": ["True", "False"],
            "correct_answer": correct_answer,
            "explanation": f"Explanation: {concept['context'][:150]}...",
            "difficulty": "easy"
        }

    def _create_short_answer_question(self, concept: Dict[str, str]) -> Dict[str, Any]:
        """
        Create a short answer question based on a concept
        """
        question_text = f"Explain the concept of {concept['text']} and its importance in robotics."

        return {
            "id": f"sa_{hash(concept['text']) % 10000}",
            "type": "short_answer",
            "question": question_text,
            "sample_answer": f"{concept['text']} refers to {concept['context'][:100]}...",
            "explanation": f"Key points about {concept['text']}: {concept['context'][:200]}...",
            "difficulty": "hard"
        }

    def _generate_multiple_choice_questions(self, content: str, count: int) -> List[Dict[str, Any]]:
        """
        Generate multiple choice questions from content
        """
        concepts = self._extract_key_concepts(content)
        questions = []

        for i in range(min(count, len(concepts))):
            concept = concepts[i]
            question = self._create_multiple_choice_question(concept)
            if question:
                questions.append(question)

        return questions

    def _generate_true_false_questions(self, content: str, count: int) -> List[Dict[str, Any]]:
        """
        Generate true/false questions from content
        """
        concepts = self._extract_key_concepts(content)
        questions = []

        for i in range(min(count, len(concepts))):
            concept = concepts[i]
            question = self._create_true_false_question(concept)
            if question:
                questions.append(question)

        return questions

    def _generate_short_answer_questions(self, content: str, count: int) -> List[Dict[str, Any]]:
        """
        Generate short answer questions from content
        """
        concepts = self._extract_key_concepts(content)
        questions = []

        for i in range(min(count, len(concepts))):
            concept = concepts[i]
            question = self._create_short_answer_question(concept)
            if question:
                questions.append(question)

        return questions

    def validate_quiz(self, quiz: Dict[str, Any]) -> Dict[str, Any]:
        """
        Validate the generated quiz
        """
        issues = []

        questions = quiz.get('questions', [])
        if not questions:
            issues.append("No questions generated")

        for i, question in enumerate(questions):
            if not question.get('question'):
                issues.append(f"Question {i+1} has no content")
            if question.get('type') == 'multiple_choice':
                options = question.get('options', [])
                if len(options) < 2:
                    issues.append(f"Multiple choice question {i+1} has fewer than 2 options")

        return {
            "is_valid": len(issues) == 0,
            "issues": issues,
            "question_count": len(questions)
        }


# Global quiz generator instance
quiz_generator = QuizGenerator()