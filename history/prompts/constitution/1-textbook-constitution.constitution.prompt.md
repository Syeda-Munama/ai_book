---
id: 1
title: Textbook Constitution Update
stage: constitution
date: 2025-12-13
surface: agent
model: claude-sonnet-4-5-20250929
feature: none
branch: master
user: user
command: /sp.constitution
labels: [constitution, textbook, robotics, ai]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
 - .specify/memory/constitution.md
tests:
 - none
---

## Prompt

```text
Project: Textbook for Teaching Physical AI & Humanoid Robotics Course

Core principles:
- Accuracy through primary source verification and up-to-date robotics knowledge (e.g., ROS 2 Humble docs, NVIDIA Isaac 2024 releases)
- Clarity for educational audience (university students with AI/CS/EE background)
- Reproducibility (all code examples tested in ROS 2, Gazebo, Isaac Sim environments)
- Rigor (prefer official docs, peer-reviewed papers on embodied AI)
- Spec-Driven Development: Content generated from high-level specs using Spec-Kit Plus
- AI-Assisted Creation: Leverage Claude/Qwen for drafts, with human edits for originality
- Interactivity: Use MDX for code snippets, Mermaid diagrams, quizzes, and embeds
- Educational Focus: Align with learning outcomes, weekly breakdown, and capstone project

Key standards:
- All factual claims traceable to sources (e.g., cite ROS tutorials, NVIDIA papers)
- Citation format: APA style for references; inline Markdown links for quick access
- Source types: Minimum 50% from official documentation (ROS, NVIDIA) and academic journals (e.g., IEEE, arXiv on humanoid robotics)
- Plagiarism check: 0% tolerance; paraphrase and customize all AI-generated text
- Writing clarity: Flesch-Kincaid grade 8-12; use simple explanations for complex topics like bipedal locomotion
- Consistency: Standard terminology (e.g., 'embodied intelligence', 'VSLAM'); uniform chapter structure (objectives, content, examples, quizzes)
- Technical Validation: Code blocks runnable; hardware reqs realistic (e.g., RTX 4070+ for Isaac Sim)
- Inclusivity: Gender-neutral language; alt text for images/diagrams; accessible MDX

Constraints:
- Chapter count: 7-8 chapters matching course modules (e.g., ROS 2, Gazebo, Isaac, VLA, Hardware)
- Word count per chapter: 2,000-4,000 words, with sections for theory, practice, and assessments
- Minimum 20 sources overall (e.g., NVIDIA docs, ROS wiki, papers on VLA like RT-2)
- Format: MDX for Docusaurus; include LaTeX for equations (e.g., kinematics)
- Tools: Spec-Kit Plus for specs, Claude Code for generation, Docusaurus for build/deploy
- Deployment: GitHub Pages or Vercel; ensure mobile-friendly
- Budget Alignment: Hardware sections reflect options (On-Prem vs. Cloud, ~$700 Jetson kit)
- Timeline: Hackathon-friendly (4-6 hours total; iterative generation)

Success criteria:
- All claims verified against sources (e.g., cross-check ROS 2 features)
- Zero plagiarism detected (run through tools like Grammarly)
- Passes fact-checking (e.g., simulate expert review via Claude evaluation)
- Book deploys with no errors; all links/images functional
- Covers all course elements: Modules, weekly breakdown, assessments, hardware reqs
- Interactive features work (e.g., code copyable, diagrams render)
- Usable for teaching: Clear path from digital AI to physical humanoid deployment
- Feedback metrics: High readability; practical for capstone (e.g., voice-to-action demo)
```

## Response snapshot

Updated the project constitution to reflect the requirements for the Physical AI & Humanoid Robotics textbook. The new constitution includes:

- 8 core principles covering accuracy, clarity, reproducibility, rigor, spec-driven development, AI-assisted creation, interactivity, and educational alignment
- Content standards for citation, writing, and technical validation
- Course structure and delivery guidelines
- Governance section establishing the document as the authoritative standard

The constitution now properly reflects the project's focus on creating educational content for university students learning about physical AI and humanoid robotics, with emphasis on using authoritative sources like ROS 2 and NVIDIA Isaac documentation.

## Outcome

- ✅ Impact: Created comprehensive constitution for robotics textbook project
- 🧪 Tests: none
- 📁 Files: Updated .specify/memory/constitution.md with new content
- 🔁 Next prompts: Ready for spec creation and content generation
- 🧠 Reflection: Constitution now serves as authoritative guide for all future content development

## Evaluation notes (flywheel)

- Failure modes observed: none
- Graders run and results (PASS/FAIL): PASS - all requirements met
- Prompt variant (if applicable): N/A
- Next experiment (smallest change to try): Begin creating chapter specifications