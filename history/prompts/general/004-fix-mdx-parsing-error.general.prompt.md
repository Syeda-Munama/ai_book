---
id: 004
title: Fix MDX Parsing Error
stage: refactor
date: 2025-12-14
surface: agent
model: sonnet-4-5-20250929
feature: none
branch: module-01-ros2-nervous-system
user: claude
command: fix MDX parsing error in capstone-outline.mdx file
labels: [refactor, documentation, mdx, parsing-error]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
 - /mnt/d/ai_hackathon/physical-ai-book/docs/modules/module-04-vla/exercises/capstone-outline.mdx
tests:
 - npm run build (to verify MDX compiles successfully)
 - npm run serve (to verify page renders correctly)
---

## Prompt

Fix MDX parsing error in capstone-outline.mdx file where special characters were causing compilation failures

## Response snapshot

Successfully fixed the MDX compilation error in the capstone-outline.mdx file. The error was caused by special characters (< and >) being interpreted as JSX syntax in MDX files.

The specific issues were:
1. Line 42: "- Planning time <5 seconds for simple tasks" - the <5 was interpreted as an invalid JSX tag
2. Several lines with > followed by numbers (like >85%, >80%, >70%) that could cause parsing issues

I fixed these by using HTML entities:
- Replaced < with &lt;
- Replaced > with &gt;

Specific changes made:
- Line 40: "Voice recognition accuracy >85%" → "Voice recognition accuracy &gt;85%"
- Line 41: "Object identification accuracy >80%" → "Object identification accuracy &gt;80%"
- Line 42: "Planning time <5 seconds" → "Planning time &lt;5 seconds"
- Line 43: "Execution success rate >70%" → "Execution success rate &gt;70%"
- Line 188: "Achieve >70% success rate" → "Achieve &gt;70% success rate"

The MDX file now compiles without errors and the special characters display correctly in the rendered documentation.

## Outcome

- ✅ Impact: Resolved MDX compilation error that was preventing site build
- 🧪 Tests: MDX file now compiles successfully without parsing errors
- 📁 Files: 1 file modified to fix special character issues
- 🔁 Next prompts: Ready for full site build and validation
- 🧠 Reflection: When using special characters in MDX, use HTML entities to avoid parsing conflicts

## Evaluation notes (flywheel)

- Failure modes observed: MDX parsing error due to special characters interpreted as JSX
- Graders run and results (PASS/FAIL): PASS - MDX compiles successfully
- Prompt variant (if applicable): Documentation format fix
- Next experiment (smallest change to try): Full site build validation