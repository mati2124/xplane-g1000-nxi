---
name: model-g1000-nxi
description: Reference the real Garmin G1000 NXi before modeling any avionics behavior, layout, page, softkey, or instrument in this project. Use whenever the user asks to model, build, add, replicate, lay out, or change a PFD/MFD feature, page group (MAP/WPT/AUX/NRST), softkey, annunciation, or any gauge so the implementation matches the actual G1000 NXi via screenshots, Garmin documentation, or the Working Title G1000 NXi project.
---

# Model the real G1000 NXi

When asked to model anything in this project, match the **actual Garmin G1000 NXi** — not a guess or a generic glass cockpit. Verify behavior and layout against a real source before writing code.

## Reference sources (in priority order)

1. **Screenshots / images** of the real unit. If the user attaches one, study it. If not and the detail is non-obvious, ask the user for a screenshot before guessing.
2. **Garmin documentation** — G1000 NXi Pilot's Guide and Cockpit Reference Guide (public PDFs). Use these for exact softkey labels, menu trees, page-group structure, field names, and annunciation wording.
3. **Working Title G1000 NXi project** (open-source MSFS mod) for interaction/behavior details when Garmin docs are ambiguous.

If you cannot confirm a detail from any source, say so and ask rather than inventing it.

## Workflow

1. Identify the exact G1000 NXi element being modeled (page, softkey path, instrument, annunciation).
2. Confirm its real layout and behavior against a source above — exact labels, positions, colors, states, and transitions.
3. Implement with vector drawing in `avionics-core` so the same code runs in both shells (see `README.md`).
4. Note in your response which source you matched against.

## Constraints

- Replicate **behavior and layout only**. Do **not** copy Garmin bitmaps, fonts, or decompiled code — reimplement the look with vector drawing.
- Match real labels and terminology **verbatim** (e.g. softkey text, page-group names like MAP/WPT/AUX/NRST). Reuse existing project constants/enums; don't introduce magic strings or ints.
- Reuse existing styles/helpers before creating new ones.
- Before committing, ensure it builds and compiles.
