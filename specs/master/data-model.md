# Data Model for Physical AI & Humanoid Robotics Textbook

## Entities

### Chapter
- **id**: string (e.g., "chapter-1", "introduction-to-physical-ai")
- **title**: string (e.g., "Foundations of Physical AI")
- **number**: integer (1-8)
- **objectives**: string[] (learning objectives)
- **lessons**: Lesson[] (exactly 3 lessons)
- **personalization_options**: PersonalizationOptions
- **urdu_translation_available**: boolean

### Lesson
- **id**: string (e.g., "lesson-1.1", "what-is-physical-ai")
- **title**: string (e.g., "What is Physical AI?")
- **chapter_id**: string
- **number**: integer (1-3 within chapter)
- **content**: string (Markdown format)
- **concepts**: string[] (key concepts covered)
- **code_examples**: CodeExample[]
- **simulation_exercises**: SimulationExercise[]
- **mental_models**: MentalModel[]
- **urdu_translation_available**: boolean

### CodeExample
- **id**: string
- **title**: string
- **language**: string (e.g., "python", "bash", "yaml")
- **code**: string
- **explanation**: string
- **related_concepts**: string[]

### SimulationExercise
- **id**: string
- **title**: string
- **description**: string
- **requirements**: string[] (software/hardware needed)
- **steps**: string[]
- **expected_outcomes**: string[]

### MentalModel
- **id**: string
- **title**: string
- **description**: string
- **diagram_reference**: string (placeholder for diagram)
- **related_concepts**: string[]

### User
- **id**: string
- **software_experience**: "beginner" | "intermediate" | "advanced"
- **robotics_experience**: "beginner" | "intermediate" | "advanced"
- **available_hardware**: "none" | "simulation_only" | "rtx_enabled" | "jetson" | "cloud" | "other"
- **preferred_language**: "English" | "Urdu"
- **personalization_enabled**: boolean
- **progress**: UserProgress

### UserProgress
- **chapter_progress**: ChapterProgress[]
- **lesson_progress**: LessonProgress[]
- **completed_lessons**: string[] (lesson IDs)
- **current_lesson**: string (lesson ID)
- **personalization_settings**: PersonalizationOptions

### ChapterProgress
- **chapter_id**: string
- **completed_lessons**: integer (count)
- **total_lessons**: integer (always 3)
- **personalization_level**: "default" | "beginner" | "intermediate" | "advanced"

### LessonProgress
- **lesson_id**: string
- **completed**: boolean
- **time_spent**: number (seconds)
- **notes**: string[]
- **questions_asked**: number

### PersonalizationOptions
- **content_depth**: "shallow" | "standard" | "deep"
- **example_complexity**: "basic" | "standard" | "advanced"
- **prerequisite_review**: boolean
- **additional_resources**: boolean

### GlossaryTerm
- **id**: string
- **term**: string (e.g., "ROS 2", "URDF", "VLA")
- **definition**: string
- **chapter_introduced**: string
- **related_terms**: string[]

## Relationships
- Chapter contains 3 Lessons (1:3)
- Lesson has many CodeExamples (1:many)
- Lesson has many SimulationExercises (1:many)
- Lesson has many MentalModels (1:many)
- User has one UserProgress (1:1)
- UserProgress contains many ChapterProgress (1:many)
- UserProgress contains many LessonProgress (1:many)

## Validation Rules
- Each Chapter must have exactly 3 Lessons
- Each Lesson must have at least one Concept
- Each Lesson must have at least one CodeExample OR SimulationExercise
- User software_experience and robotics_experience must be one of the defined values
- Chapter number must be between 1 and 8 (inclusive)
- Lesson number must be between 1 and 3 (inclusive)
- Content must be in valid Markdown format
- All URIs and file paths must be valid

## State Transitions
- Lesson: draft → review → published → archived
- Chapter: draft → review → published → archived
- UserProgress: not_started → in_progress → completed