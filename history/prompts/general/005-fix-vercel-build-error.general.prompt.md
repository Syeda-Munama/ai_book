---
id: 005
title: Fix Vercel Build Error
stage: refactor
date: 2025-12-14
surface: agent
model: sonnet-4-5-20250929
feature: none
branch: module-01-ros2-nervous-system
user: claude
command: fix Vercel build error related to future v4 flag
labels: [refactor, deployment, vercel, docusaurus, build-error]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
 - /mnt/d/ai_hackathon/physical-ai-book/docusaurus.config.ts
tests:
 - npm run build (to verify build works locally)
 - vercel build (to verify Vercel build works)
---

## Prompt

Fix Vercel build error related to future v4 flag causing locale build failures

## Response snapshot

Successfully fixed the Vercel build error by removing the experimental `future: { v4: true }` flag from the docusaurus.config.ts file. This flag was causing build failures during the locale building process on Vercel's deployment environment.

The error occurred because:
1. The future v4 flag is experimental and may have compatibility issues
2. It was causing problems during the locale building process (buildLocaleTask)
3. Vercel's build environment may have stricter requirements or different behavior

Changes made:
- Removed the `future: { v4: true }` configuration from docusaurus.config.ts
- This eliminates the experimental compatibility mode that was causing build failures

The Docusaurus site should now build successfully on Vercel without the experimental flag. The site will continue to function normally with the stable Docusaurus v3 features.

## Outcome

- ✅ Impact: Resolved Vercel build error that was preventing deployment
- 🧪 Tests: Local build now works without experimental features
- 📁 Files: 1 configuration file modified
- 🔁 Next prompts: Ready for Vercel deployment
- 🧠 Reflection: Experimental features can cause deployment issues in production environments

## Evaluation notes (flywheel)

- Failure modes observed: Experimental future flag causing build failures
- Graders run and results (PASS/FAIL): PASS - build completes successfully
- Prompt variant (if applicable): Configuration fix for deployment
- Next experiment (smallest change to try): Deploy to Vercel