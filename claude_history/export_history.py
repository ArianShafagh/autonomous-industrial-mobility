#!/usr/bin/env python3
"""Turn Claude Code session logs (.jsonl) into readable Markdown conversations.

    python3 claude_history/export_history.py            # re-export everything in raw/

For each raw/<session>.jsonl this writes <date>_<session-short>.md with every message you typed
and every answer Claude gave, in order. Tool calls are listed by name and short description only
(the full commands and their output stay in the raw file), so a conversation reads like a chat.
Automatic system notices (task notifications, reminders) are left out.
"""
import glob
import json
import os
import re

HERE = os.path.dirname(os.path.abspath(__file__))
RAW = os.path.join(HERE, "raw")

# Blocks the harness injects into messages; not typed by anyone, so not part of the conversation.
NOISE = re.compile(r"<(system-reminder|task-notification|local-command-stdout|local-command-caveat|"
                   r"command-name|command-message|command-args)>.*?</\1>", re.S)


def text_of(content):
    """(text, [tool uses]) from a message's content, whether it is a string or a list of blocks."""
    if isinstance(content, str):
        return content, []
    texts, tools = [], []
    for block in content or []:
        kind = block.get("type")
        if kind == "text":
            texts.append(block.get("text", ""))
        elif kind == "tool_use":
            inp = block.get("input", {}) or {}
            desc = inp.get("description") or inp.get("file_path") or inp.get("skill") or ""
            tools.append(f"{block.get('name')}: {desc}".strip(": "))
    return "\n".join(texts), tools


def export(path):
    entries = []
    for line in open(path, encoding="utf-8"):
        try:
            entries.append(json.loads(line))
        except json.JSONDecodeError:
            continue
    stamps = [e["timestamp"] for e in entries if e.get("timestamp")]
    session = os.path.basename(path)[:-6]
    first_day = stamps[0][:10] if stamps else "unknown"
    out = [f"# Claude Code session {session[:8]}", "",
           f"{stamps[0][:16].replace('T', ' ')} → {stamps[-1][:16].replace('T', ' ')} (UTC)"
           if stamps else "", "",
           "Readable export of `raw/" + os.path.basename(path) + "`: your messages and Claude's "
           "answers in order. Tool calls are listed by name only; the raw file has everything.", ""]
    pending_tools = []
    for e in entries:
        kind = e.get("type")
        msg = e.get("message") or {}
        if kind == "user":
            content = msg.get("content")
            # Tool results come back as "user" entries; they are not something anyone typed.
            if isinstance(content, list) and all(b.get("type") == "tool_result" for b in content):
                continue
            text, _ = text_of(content)
            text = NOISE.sub("", text).strip()
            if not text or e.get("isMeta") or text.startswith("This session is being continued"):
                if text.startswith("This session is being continued"):
                    out += ["---", "", "*(context was compacted here; Claude continued from a "
                            "summary of the conversation so far)*", ""]
                continue
            if pending_tools:
                out += ["<details><summary>tools used (" + str(len(pending_tools)) + ")</summary>",
                        "", *[f"- {t}" for t in pending_tools], "", "</details>", ""]
                pending_tools = []
            when = (e.get("timestamp") or "")[:16].replace("T", " ")
            out += [f"## You — {when}", "", text, ""]
        elif kind == "assistant":
            text, tools = text_of(msg.get("content"))
            pending_tools += tools
            text = text.strip()
            if text:
                out += ["**Claude:**", "", text, ""]
    if pending_tools:
        out += ["<details><summary>tools used (" + str(len(pending_tools)) + ")</summary>", "",
                *[f"- {t}" for t in pending_tools], "", "</details>", ""]
    name = os.path.join(HERE, f"{first_day}_{session[:8]}.md")
    with open(name, "w", encoding="utf-8") as fh:
        fh.write("\n".join(out))
    return name


if __name__ == "__main__":
    for path in sorted(glob.glob(os.path.join(RAW, "*.jsonl"))):
        print(os.path.relpath(export(path), HERE))
