---
name: maintain-project-docs
description: Use in /home/fuzzrobo/uraki_ws when implementation or investigation creates deferred work, establishes a gng_vlut_system specification, or changes documented behavior; update only the relevant task candidate, technical specification, or release note while preserving current user edits.
---

# Maintain Project Docs

Keep project documentation aligned with the work without loading or rewriting unrelated documents.

## Workflow

1. Use `rg` to locate the relevant heading or symbol before reading a document.
2. Read the current on-disk target file immediately before editing it.
3. Preserve manual edits and make the smallest applicable patch.
4. Route the update:
   - Unimplemented or unsettled work: append one concise item to `gng_vlut_system/docs/TASK_CANDIDATES.md`.
   - Implemented and stable behavior: update only the affected variables, topics, or flow in `gng_vlut_system/docs/TECHNICAL_SPEC.md`.
   - Code or configuration behavior changed: add one entry under `gng_vlut_system/docs/releases/` using `RELEASE_NOTE_TEMPLATE.md`.
5. When a candidate is implemented, remove only that completed candidate after its stable behavior is documented.
6. Review the final diff and never restore text that the user removed or rewrote.

Do not create documentation churn for investigation-only work unless it identifies a concrete deferred task.
