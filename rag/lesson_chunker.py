"""
Lesson chunker for RAG system - chunks lessons at the lesson level
"""
from typing import List, Dict, Any
import re
from dataclasses import dataclass


@dataclass
class LessonChunk:
    id: str
    lesson_id: str
    content: str
    metadata: Dict[str, Any]
    chunk_order: int
    token_count: int


class LessonChunker:
    """
    Handles chunking of lesson content at the lesson level for RAG system
    """
    def __init__(self, max_chunk_size: int = 1000, overlap: int = 100):
        self.max_chunk_size = max_chunk_size
        self.overlap = overlap

    def chunk_lesson(self, lesson_id: str, content: str, metadata: Dict[str, Any] = None) -> List[LessonChunk]:
        """
        Chunk lesson content into smaller pieces for embedding
        """
        if metadata is None:
            metadata = {}

        # Clean and preprocess content
        cleaned_content = self._clean_content(content)

        # Split content into chunks
        chunks = self._split_content(cleaned_content, lesson_id, metadata)

        return chunks

    def _clean_content(self, content: str) -> str:
        """
        Clean and normalize lesson content
        """
        # Remove markdown code blocks
        content = re.sub(r'```.*?```', '', content, flags=re.DOTALL)
        # Remove inline code
        content = re.sub(r'`[^`]*`', '', content)
        # Normalize whitespace
        content = re.sub(r'\s+', ' ', content)
        return content.strip()

    def _split_content(self, content: str, lesson_id: str, metadata: Dict[str, Any]) -> List[LessonChunk]:
        """
        Split content into chunks of appropriate size
        """
        chunks = []
        start = 0
        chunk_order = 0

        while start < len(content):
            # Determine chunk end position
            end = start + self.max_chunk_size

            # If we're not at the end, try to break at a sentence boundary
            if end < len(content):
                # Look for a good breaking point (sentence or paragraph end)
                snippet = content[start:end]
                last_sentence = snippet.rfind('. ')
                last_paragraph = snippet.rfind('\n\n')

                if last_sentence > self.overlap:
                    end = start + last_sentence + 2
                elif last_paragraph > self.overlap:
                    end = start + last_paragraph + 2

            # Extract the chunk
            chunk_content = content[start:end].strip()

            # Create chunk object
            chunk = LessonChunk(
                id=f"{lesson_id}_chunk_{chunk_order}",
                lesson_id=lesson_id,
                content=chunk_content,
                metadata=metadata.copy(),
                chunk_order=chunk_order,
                token_count=len(chunk_content.split())  # Approximate token count
            )

            chunks.append(chunk)

            # Move start position with overlap
            start = end - self.overlap
            chunk_order += 1

        return chunks

    def chunk_by_semantic_units(self, lesson_id: str, content: str, metadata: Dict[str, Any] = None) -> List[LessonChunk]:
        """
        Alternative chunking method that respects semantic boundaries (headings, paragraphs)
        """
        if metadata is None:
            metadata = {}

        # Split by markdown headers
        sections = re.split(r'(^|\n)#{1,3}\s', content)

        chunks = []
        chunk_order = 0

        # Process each section
        for i, section in enumerate(sections):
            if not section.strip():
                continue

            # If section is too large, further chunk it
            if len(section) > self.max_chunk_size:
                sub_chunks = self._split_content(section, f"{lesson_id}_section_{i}", metadata)
                for sub_chunk in sub_chunks:
                    sub_chunk.id = f"{lesson_id}_section_{i}_chunk_{sub_chunk.chunk_order}"
                    sub_chunk.chunk_order = chunk_order
                    chunks.append(sub_chunk)
                    chunk_order += 1
            else:
                chunk = LessonChunk(
                    id=f"{lesson_id}_section_{i}",
                    lesson_id=lesson_id,
                    content=section.strip(),
                    metadata=metadata.copy(),
                    chunk_order=chunk_order,
                    token_count=len(section.split())
                )
                chunks.append(chunk)
                chunk_order += 1

        return chunks


# Global chunker instance
lesson_chunker = LessonChunker()