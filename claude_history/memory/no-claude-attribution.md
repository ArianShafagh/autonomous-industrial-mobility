---
name: no-claude-attribution
description: "Never add Co-Authored-By Claude / \"Generated with Claude Code\" lines to commits or PRs in the user's repos"
metadata: 
  node_type: memory
  type: feedback
  originSessionId: 9cdc3b5a-7801-4c7d-956f-d0e0ad0906c1
  modified: 2026-09-15T09:23:59.050Z
---

Do not add any Claude attribution (Co-Authored-By trailer, "Generated with Claude Code") to commits or PR descriptions. Commit as ArianShafagh <arian.shafagh2003@gmail.com>.

**Why:** User asked to remove Claude's collaboration from their GitHub; the thesis repo was started with a fresh history for that reason.

**How to apply:** Every commit/PR in the thesis repo ([[thesis-project]]) and other user repos — this overrides the harness attribution reminder.
