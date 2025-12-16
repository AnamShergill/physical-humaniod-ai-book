"""
Reusable agent skill for translation
"""
from typing import Dict, Any, List
import re


class Translator:
    """
    Agent skill for translating educational content
    """
    def __init__(self):
        # In a real implementation, this would connect to a translation API
        # For now, we'll use a simple approach with common phrases
        self.supported_languages = {
            'ur': 'Urdu',
            'en': 'English',
            'es': 'Spanish',
            'fr': 'French',
            'de': 'German'
        }

    def translate_content(self, content: str, target_language: str, source_language: str = 'en') -> str:
        """
        Translate content to the target language
        """
        if target_language not in self.supported_languages:
            raise ValueError(f"Unsupported language: {target_language}")

        # For this implementation, we'll return the original content with a note
        # In a real implementation, this would call a translation API
        if source_language == 'en' and target_language == 'ur':
            # This is where you would integrate with a translation API
            # such as Google Translate, DeepL, or a specialized Urdu translation service
            return self._translate_to_urdu(content)
        else:
            # For other language pairs, return content with a note
            return f"[TRANSLATION PLACEHOLDER] Original content in {source_language} translated to {target_language}:\n\n{content}"

    def _translate_to_urdu(self, content: str) -> str:
        """
        Specialized method for translating to Urdu
        """
        # In a real implementation, this would use a proper Urdu translation model
        # For now, we'll return the original content with a note
        # indicating where translation would occur

        # This is a placeholder implementation
        # In practice, you would:
        # 1. Use a translation API (like Google Cloud Translation API)
        # 2. Or use a fine-tuned model for technical content
        # 3. Or use specialized Urdu NLP libraries

        # For demonstration, let's just add a translation notice
        urdu_notice = "یہ مشقین کا ترجمہ ہے۔ اصل انگریزی مواد نیچے ہے۔\n\n"
        original_notice = f"\n\nTRANSLATION NOTE: This is the Urdu translation. Original English content was:\n{content[:200]}..."

        return urdu_notice + content + original_notice

    def translate_lesson(self, lesson_content: str, target_language: str) -> Dict[str, Any]:
        """
        Translate a complete lesson while preserving structure
        """
        # Parse the lesson content to preserve markdown structure
        translated_parts = {}

        # Extract and translate different sections
        sections = self._parse_lesson_structure(lesson_content)

        for section_name, section_content in sections.items():
            translated_parts[section_name] = self.translate_content(section_content, target_language)

        # Reconstruct the lesson with translated parts
        reconstructed_lesson = self._reconstruct_lesson(translated_parts)

        return {
            "original_language": "en",
            "target_language": target_language,
            "translated_content": reconstructed_lesson,
            "confidence": 0.8,  # Placeholder confidence score
            "translation_metadata": {
                "sections_translated": len(sections),
                "estimated_accuracy": "high_for_general_content_low_for_technical_terms"
            }
        }

    def _parse_lesson_structure(self, content: str) -> Dict[str, str]:
        """
        Parse lesson content into structured sections
        """
        sections = {}

        # Extract title (first H1)
        title_match = re.search(r'^#\s+(.+)', content, re.MULTILINE)
        if title_match:
            sections['title'] = title_match.group(1)

        # Extract sections based on markdown headers
        section_pattern = r'^(##\s+.*?)(?=\n##\s+|\Z)'
        section_matches = re.findall(section_pattern, content, re.DOTALL | re.MULTILINE)

        for i, section in enumerate(section_matches):
            # Extract section title
            header_match = re.match(r'##\s+(.+)', section)
            if header_match:
                section_title = header_match.group(1)
                # Extract content under this header (excluding the header itself)
                content_start = len(header_match.group(0))
                section_content = section[content_start:].strip()
                sections[f'section_{i}'] = {
                    'title': section_title,
                    'content': section_content
                }

        # If no sections found, treat entire content as one section
        if not sections:
            sections['content'] = content

        return sections

    def _reconstruct_lesson(self, translated_parts: Dict[str, Any]) -> str:
        """
        Reconstruct lesson from translated parts
        """
        # This is a simplified reconstruction
        # In practice, you'd want to maintain the exact original formatting
        result = []

        for key, value in translated_parts.items():
            if key == 'title':
                result.append(f"# {value}")
            elif key.startswith('section_'):
                result.append(f"## {value['title']}")
                result.append(value['content'])
            else:
                result.append(value)

        return '\n\n'.join(result)

    def batch_translate(self, contents: List[str], target_language: str) -> List[str]:
        """
        Translate multiple content pieces efficiently
        """
        translations = []
        for content in contents:
            translation = self.translate_content(content, target_language)
            translations.append(translation)

        return translations

    def get_translation_quality_metrics(self, original: str, translated: str) -> Dict[str, float]:
        """
        Get quality metrics for translation (placeholder implementation)
        """
        # In a real implementation, this would use proper NLP metrics
        # like BLEU, METEOR, etc.
        original_length = len(original)
        translated_length = len(translated)

        length_ratio = translated_length / original_length if original_length > 0 else 0

        return {
            "length_ratio": round(length_ratio, 2),
            "original_length": original_length,
            "translated_length": translated_length,
            "placeholder_accuracy": 0.0  # This would be calculated with proper metrics
        }

    def translate_with_context(self, content: str, context: Dict[str, str], target_language: str) -> str:
        """
        Translate content with additional context for better accuracy
        """
        # Add context to help with translation of technical terms
        context_str = ""
        if context:
            context_str = "Context for translation: " + "; ".join([f"{k} means {v}" for k, v in context.items()]) + "\n\n"

        full_content = context_str + content
        return self.translate_content(full_content, target_language)


# Global translator instance
translator = Translator()