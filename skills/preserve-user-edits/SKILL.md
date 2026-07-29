---
name: preserve-user-edits
description: Use when editing existing workspace files that may contain manual user changes; preserve the current on-disk content and never restore older model-generated content unless the user explicitly asks for that rollback.
---

# Preserve User Edits

## Core rule

When editing an existing file, treat the current file on disk as the source of truth.

Do not:

- restore a prior version just because it matches an earlier assistant state
- reapply text from memory over user edits
- overwrite sections that the user has manually changed unless the user asked for that exact replacement

## Required workflow

1. Read the current file contents first.
2. Compare against the latest diff or visible working tree state when available.
3. Preserve any user-added or user-modified lines.
4. Make the smallest patch that satisfies the request.
5. If the current file already diverges from the version you expected, continue from the current file instead of "fixing" it back.

## If conflict appears

If a requested edit would destroy a user change or would require guessing which version to keep, stop and report the conflict instead of silently reverting anything.

## Typical trigger cases

- markdown or text files the user may hand-edit
- docs that are being iterated on in the same workspace
- any file where a later assistant turn must append to the latest user-edited state
