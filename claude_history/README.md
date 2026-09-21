# Claude Code history of this project

The complete conversations with Claude Code that built and analysed this project, exported on
2026-09-21.

| readable conversation | raw log | what it was |
|---|---|---|
| `2026-09-15_9cdc3b5a.md` | `raw/9cdc3b5a-….jsonl` | **the main build session, 15–21 Sep**: WP0–WP9, the evaluation, the Gazebo runs, the planner check and the follow-up |
| `2026-09-17_67802d63.md` | `raw/67802d63-….jsonl` | learning session: going through the project to understand it (likely the origin of `docs/explained/`) |
| `2026-09-17_3c2b07d7.md` | `raw/3c2b07d7-….jsonl` | the `/graphify` session |
| `2026-09-20_0c77cf3f.md` | `raw/0c77cf3f-….jsonl` | analysis of `HANDOVER.md` |

- **Readable `.md` files:** every message you typed and every answer Claude gave, in order. Tool calls are listed by name only, folded under "tools used". Read these on any machine.
- **`raw/*.jsonl`:** the original session logs with everything, including every command and its full output. One JSON object per line.
- **`memory/`:** Claude Code's memory notes for this project (the rules it follows, e.g. no Claude attribution in commits). To use them on a new machine, copy them to `~/.claude/projects/<workspace-path-with-dashes>/memory/`.

Re-export after new sessions: copy the new `.jsonl` files from `~/.claude/projects/<workspace>/` into `raw/`, then run `python3 claude_history/export_history.py`.

For the *results*, start with `THESIS_DATA.md` and `HANDOVER.md` in the repository root, not with these conversations: the conversations show how things were reached, including dead ends later corrected.
