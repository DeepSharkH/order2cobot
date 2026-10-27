# Session trace — instructions for the Claude agent that ran the RJ45 session

Paste this to the Claude Code session that did the port work (same chat, so it still has
the history). Copy `session_trace_to_drawio.py` and `trace_template.json` next to it.

---

We want an honest picture of **how this session actually went**, not the ideal workflow.
Reconstruct it from this chat: what we asked you, how you understood it, what you did,
what came back, and where it went wrong. We will use it to find which of your
assumptions and missing tools made the insertion turn into blind poking.

1. Copy `trace_template.json` to `session_trace_<date>.json` and fill it from this chat.
2. Split the chat into **episodes**: one per user request, or one per distinct attempt
   when you retried on your own (e.g. "peck grid round 2"). Keep time order. 10–25
   episodes is the right size; merge trivial ones.
3. Per episode:
   - `ask`: the user's words, quoted or closely paraphrased. If you acted on your own
     initiative, say so and set `trigger`.
   - `interpretation`: what you thought was being asked and the plan you chose. Name an
     alternative you rejected if there was one.
   - `actions`: tool calls, scripts written or run, robot programs (what motion/force).
     Short, in order.
   - `observed`: the data you got back (numbers, image judgements, log lines, errors,
     protective-stop codes).
   - `outcome.status`: `success`, `partial`, `failure` or `aborted`. Add `cost`
     (wall time, bridge steps, protective stops) where you can tell.
   - `assumptions`: what you relied on. `verified` if you checked it, `unverified` if you
     didn't, `false` if it later turned out wrong. **Be complete here — this is the
     most important field.** Include geometry (tip offset, face y, port z), frames,
     calibration quality, sensor readings meaning contact, the switch staying put, what
     a camera image showed.
   - `human`: anything the operator did or said that changed the course (corrections,
     physical fixes, stop recovery, "that's cheating"). Empty if nothing.
4. Fill `summary`: where the time went, failure patterns that repeated, missing tools or
   information that would have avoided a failure, and what worked.
5. Rules:
   - Facts from this chat only. If you don't remember, write "unknown"; do not guess.
   - Don't polish: failures and dead ends are the point.
   - Don't describe the target architecture or how it *should* work.
   - Keep each box short (≤ ~6 lines of ~40 chars; actions ≤ ~10 bullets). The boxes have
     a fixed size. Put long detail in an extra `"notes"` field (ignored by the diagram).
6. Generate and check the diagram:
   ```
   python3 session_trace_to_drawio.py session_trace_<date>.json
   ```
   This writes `session_trace_<date>.drawio.xml` (open in app.diagrams.net). Send back
   both the JSON and the XML.
