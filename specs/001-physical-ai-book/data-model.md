# Data Model: Physical AI and Robotics Book

## Entities

### Chapter
- **name**: string - The name of the chapter
- **number**: integer - Sequential chapter number
- **title**: string - Full title of the chapter
- **content**: string - The chapter content in MDX format
- **wordCount**: integer - Number of words in the chapter (1500-2500)
- **diagrams**: array - List of diagram references in the chapter
- **codeSnippets**: array - List of code examples in the chapter
- **learningObjectives**: array - List of learning objectives for the chapter

### CodeSnippet
- **id**: string - Unique identifier for the snippet
- **language**: string - Programming language (python, bash, etc.)
- **code**: string - The actual code content
- **description**: string - Explanation of what the code does
- **testable**: boolean - Whether the code can be tested in ROS 2 Humble
- **chapterRef**: string - Reference to the chapter containing this snippet

### Diagram
- **id**: string - Unique identifier for the diagram
- **type**: string - Type of diagram (flowchart, architecture, URDF tree, etc.)
- **title**: string - Title of the diagram
- **description**: string - Explanation of the diagram
- **filePath**: string - Path to the diagram file
- **chapterRef**: string - Reference to the chapter containing this diagram

### Quiz
- **id**: string - Unique identifier for the quiz
- **title**: string - Title of the quiz
- **questions**: array - List of questions in the quiz
- **chapterRef**: string - Reference to the chapter this quiz is for

### Question
- **id**: string - Unique identifier for the question
- **type**: string - Type of question (multiple-choice, true-false, etc.)
- **text**: string - The question text
- **options**: array - List of possible answers (for multiple choice)
- **correctAnswer**: string - The correct answer
- **explanation**: string - Explanation of the correct answer

### Exercise
- **id**: string - Unique identifier for the exercise
- **title**: string - Title of the exercise
- **description**: string - Detailed description of the exercise
- **requirements**: array - List of requirements to complete the exercise
- **expectedOutcome**: string - What the student should achieve
- **difficulty**: string - Difficulty level (beginner, intermediate, advanced)

## Relationships

- Chapter contains many CodeSnippet
- Chapter contains many Diagram
- Chapter has zero or one Quiz
- Chapter has zero or many Exercise
- Quiz contains many Question