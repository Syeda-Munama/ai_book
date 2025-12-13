# Data Model: Physical AI & Humanoid Robotics Textbook

## Content Structure

### Module Entity
- **id**: Unique identifier (e.g., "module-01-ros2-nervous-system")
- **title**: Display title of the module
- **description**: Brief overview of the module content
- **target_audience**: Who the module is designed for (e.g., "University students", "Advanced students")
- **duration**: Estimated time to complete (in weeks or hours)
- **prerequisites**: List of required knowledge or modules
- **chapters**: Array of Chapter entities
- **assessments**: Array of Assessment entities
- **exercises**: Array of Exercise entities
- **capstone_demo**: Capstone demonstration requirements

### Chapter Entity
- **id**: Unique identifier (e.g., "chapter-01-voice-speech-integration")
- **title**: Display title of the chapter
- **module_id**: Reference to parent Module
- **word_count**: Estimated word count
- **learning_objectives**: Array of learning objectives
- **content_type**: Type of content ("mdx", "tutorial", "reference")
- **sections**: Array of Section entities
- **diagrams**: Array of Diagram entities
- **code_snippets**: Array of CodeSnippet entities
- **citations**: Array of Citation entities

### Section Entity
- **id**: Unique identifier
- **title**: Section title
- **chapter_id**: Reference to parent Chapter
- **content**: The actual content (markdown/MDX format)
- **order**: Display order within the chapter
- **type**: Content type ("theory", "practical", "example", "exercise")

### Assessment Entity
- **id**: Unique identifier
- **module_id**: Reference to parent Module
- **type**: Assessment type ("quiz", "exam", "project")
- **questions**: Array of Question entities
- **passing_score**: Minimum score required to pass
- **time_limit**: Time limit for completion (if applicable)

### Question Entity
- **id**: Unique identifier
- **assessment_id**: Reference to parent Assessment
- **question_text**: The actual question
- **question_type**: Type ("multiple_choice", "short_answer", "practical")
- **options**: Array of options (for multiple choice)
- **correct_answer**: The correct answer
- **explanation**: Explanation of the correct answer

### Exercise Entity
- **id**: Unique identifier
- **module_id**: Reference to parent Module
- **title**: Exercise title
- **description**: Detailed description of the exercise
- **difficulty**: Difficulty level ("beginner", "intermediate", "advanced")
- **estimated_time**: Time to complete the exercise
- **requirements**: List of requirements to complete the exercise
- **steps**: Array of Step entities
- **expected_outcome**: What the student should achieve

### Step Entity
- **id**: Unique identifier
- **exercise_id**: Reference to parent Exercise
- **step_number**: Order of the step
- **description**: What to do in this step
- **expected_result**: What should happen after completing this step

### CodeSnippet Entity
- **id**: Unique identifier
- **chapter_id**: Reference to parent Chapter
- **language**: Programming language ("python", "bash", "yaml", etc.)
- **code**: The actual code content
- **description**: What the code does
- **test_environment**: Environment where it should run (e.g., "ROS 2 Humble", "Ubuntu 22.04")
- **test_result**: Expected result when running the code
- **is_tested**: Boolean indicating if it has been tested in Docker

### Diagram Entity
- **id**: Unique identifier
- **chapter_id**: Reference to parent Chapter
- **type**: Diagram type ("mermaid", "screenshot", "illustration")
- **title**: Diagram title
- **description**: What the diagram shows
- **source**: Source of the diagram (Mermaid code, image path)
- **alt_text**: Alternative text for accessibility

### Citation Entity
- **id**: Unique identifier
- **chapter_id**: Reference to parent Chapter
- **type**: Citation type ("academic_paper", "documentation", "website")
- **apa_format**: Full citation in APA format
- **url**: URL if available
- **access_date**: Date when the resource was accessed
- **relevance**: How this citation relates to the content

## Content Relationships

### Module contains:
- 1 to many Chapters
- 1 Assessment (quiz)
- 2 to 3 Exercises
- 1 Capstone demo

### Chapter contains:
- 1 to many Sections
- 4+ Diagrams (as per requirements)
- 0 to many CodeSnippets
- 0 to many Citations

### Assessment contains:
- 5+ Questions (for quizzes)
- 1 passing score requirement

### Exercise contains:
- 1 to many Steps
- 1 difficulty level
- 1 estimated completion time

## Validation Rules

### Module Validation:
- Title must be 10-100 characters
- Description must be 50-500 characters
- Must have 1-4 chapters (as per different module requirements)
- Duration must be specified
- Target audience must be defined

### Chapter Validation:
- Title must be 10-100 characters
- Word count must be within specified range (1500-4000 depending on module)
- Must have at least 1 learning objective
- Must include required number of diagrams (4-5 per chapter)
- All code snippets must be tested and validated

### Assessment Validation:
- Must have at least 5 questions for quizzes
- Passing score must be between 50-100%
- All questions must have correct answers and explanations

### Exercise Validation:
- Title must be descriptive
- Difficulty level must be specified
- Estimated time must be realistic
- Requirements must be achievable with available tools

## State Transitions

### Content Development States:
1. **Draft** → Content created but not reviewed
2. **In Review** → Content under review by team
3. **Changes Requested** → Feedback received, changes needed
4. **Reviewed** → Content approved by reviewer
5. **Tested** → Code snippets tested in Docker environment
6. **Published** → Content deployed to live site

### Assessment States:
1. **Created** → Assessment questions created
2. **Validated** → Questions reviewed for accuracy
3. **Piloted** → Tested with small group of users
4. **Approved** → Ready for use in modules