#!/usr/bin/env python3
"""
Script to analyze reading time for Physical AI & Humanoid Robotics textbook
Calculates estimated reading time for all lessons to ensure < 45 minutes total
"""

import os
import re
import math
from pathlib import Path

def count_words(text):
    """Count words in text, excluding code blocks and markdown syntax"""
    # Remove code blocks
    text = re.sub(r'```.*?```', '', text, flags=re.DOTALL)
    # Remove inline code
    text = re.sub(r'`[^`]*`', '', text)
    # Remove markdown headers and list markers
    text = re.sub(r'^#+\s*', '', text, flags=re.MULTILINE)
    text = re.sub(r'^\d+\.\s*', '', text, flags=re.MULTILINE)
    text = re.sub(r'^-\s*', '', text, flags=re.MULTILINE)

    # Count remaining words
    words = re.findall(r'\b\w+\b', text)
    return len(words)

def analyze_lesson(file_path):
    """Analyze a single lesson file"""
    with open(file_path, 'r', encoding='utf-8') as f:
        content = f.read()

    word_count = count_words(content)

    # Average reading speed: 200-250 words per minute
    # Using 225 as average for technical content
    reading_time_minutes = word_count / 225.0

    return {
        'file_path': file_path,
        'word_count': word_count,
        'reading_time_minutes': round(reading_time_minutes, 2),
        'reading_time_seconds': round(reading_time_minutes * 60)
    }

def analyze_all_lessons(root_dir="ai-book/docs/chapters"):
    """Analyze all lesson files in the textbook"""
    lesson_files = []

    for root, dirs, files in os.walk(root_dir):
        for file in files:
            if file.endswith('.md'):
                lesson_files.append(os.path.join(root, file))

    results = []
    total_reading_time = 0

    print("Lesson Reading Time Analysis")
    print("=" * 60)

    for file_path in lesson_files:
        analysis = analyze_lesson(file_path)
        results.append(analysis)
        total_reading_time += analysis['reading_time_minutes']

        print(f"File: {analysis['file_path']}")
        print(f"  Words: {analysis['word_count']}")
        print(f"  Reading Time: {analysis['reading_time_minutes']} min ({analysis['reading_time_seconds']} sec)")
        print()

    print("=" * 60)
    print(f"Total Lessons: {len(results)}")
    print(f"Total Word Count: {sum(r['word_count'] for r in results)}")
    print(f"Total Reading Time: {round(total_reading_time, 2)} minutes")
    print(f"Average Reading Time: {round(total_reading_time / len(results), 2)} minutes per lesson")

    if total_reading_time <= 45:
        print(f"[SUCCESS] Reading time requirement met: {round(total_reading_time, 2)} minutes < 45 minutes")
    else:
        print(f"[FAILURE] Reading time requirement exceeded: {round(total_reading_time, 2)} minutes > 45 minutes")
        print("Consider reducing content or splitting longer lessons.")

    return results, total_reading_time

def create_reading_time_metadata():
    """Create metadata for each lesson with reading time information"""
    results, total_time = analyze_all_lessons()

    # Create a summary file with reading times
    summary_content = f"""# Textbook Reading Time Summary

Total Lessons: {len(results)}
Total Reading Time: {round(total_reading_time, 2)} minutes
Target Reading Time: < 45 minutes

## Individual Lesson Reading Times

"""

    for result in sorted(results, key=lambda x: x['reading_time_minutes'], reverse=True):
        rel_path = Path(result['file_path']).relative_to(Path('ai-book/docs'))
        summary_content += f"- {rel_path}: {result['reading_time_minutes']} min ({result['word_count']} words)\n"

    with open('ai-book/docs/reading_time_summary.md', 'w', encoding='utf-8') as f:
        f.write(summary_content)

    print(f"\nReading time summary saved to: ai-book/docs/reading_time_summary.md")

    return results

if __name__ == "__main__":
    results, total_time = analyze_all_lessons()

    # Also create metadata file
    create_reading_time_metadata()

    print(f"\nAnalysis complete! Total reading time: {round(total_time, 2)} minutes")