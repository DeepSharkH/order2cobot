#!/usr/bin/env python3
"""Turn a session trace (JSON) into a draw.io timeline.

Usage: python3 session_trace_to_drawio.py trace.json [out.xml]

Columns = episodes in time order. Rows = what the user asked, how Claude read it,
what it did, what came back, the outcome, the assumptions it relied on, and any
human help. Stdlib only, so it runs on the Jetson or any laptop.
"""
import json
import sys
from xml.sax.saxutils import escape

LANES = [  # key in episode, title, fill, stroke
    ("ask", "User asked", "#f5f5f5", "#666666"),
    ("interpretation", "Claude understood / planned", "#d5e8d4", "#82b366"),
    ("actions", "Claude did (tools, scripts, robot programs)", "#dae8fc", "#6c8ebf"),
    ("observed", "What came back (data, images, logs, errors)", "#e1d5e7", "#9673a6"),
    ("outcome", "Outcome", "#ffffff", "#333333"),
    ("assumptions", "Assumptions relied on", "#f8cecc", "#b85450"),
    ("human", "Human intervention / correction", "#fff2cc", "#d6b656"),
]
OUTCOME_COLORS = {
    "success": ("#d5e8d4", "#82b366"),
    "partial": ("#fff2cc", "#d6b656"),
    "failure": ("#f8cecc", "#b85450"),
    "aborted": ("#e6e6e6", "#666666"),
}
VERDICT_ICON = {"verified": "✓", "unverified": "?", "false": "✗"}

LABEL_W, COL_W, BOX_W, X0, Y0 = 170, 250, 230, 20, 130
LANE_H = {"ask": 120, "interpretation": 130, "actions": 190, "observed": 150,
          "outcome": 110, "assumptions": 150, "human": 110}


def html(text):
    """Plain text with newlines -> draw.io html label (escaped later)."""
    return "<br>".join(escape(line) for line in str(text).split("\n"))


def bullets(items):
    return "<br>".join("• " + escape(str(i)) for i in items)


def lane_content(ep, key):
    v = ep.get(key)
    if not v:
        return ""
    if key == "actions":
        return bullets(v)
    if key == "observed":
        return bullets(v) if isinstance(v, list) else html(v)
    if key == "outcome":
        o = v
        cost = o.get("cost", {})
        cost_s = " · ".join(f"{k}: {val}" for k, val in cost.items())
        return (f"<b>{escape(o.get('status', '?').upper())}</b><br>{html(o.get('summary', ''))}"
                + (f"<br><i>{escape(cost_s)}</i>" if cost_s else ""))
    if key == "assumptions":
        return "<br>".join(
            f"{VERDICT_ICON.get(a.get('status'), '?')} {escape(a.get('text', ''))}" for a in v)
    if key == "ask":
        return f"<i>“{html(v)}”</i>"
    return html(v)


def build(trace):
    eps = trace["episodes"]
    cells = []

    def vertex(cid, value, style, x, y, w, h):
        cells.append(
            f'<mxCell id="{cid}" value="{escape(value, {chr(34): "&quot;"})}" style="{style}" '
            f'vertex="1" parent="1"><mxGeometry x="{x}" y="{y}" width="{w}" height="{h}" as="geometry" /></mxCell>')

    width = LABEL_W + 20 + len(eps) * COL_W
    meta = trace.get("session", {})
    title = f"<b>Session trace — {escape(meta.get('title', ''))}</b> · {escape(meta.get('date', ''))}"
    if meta.get("goal"):
        title += f"<br>Goal: {escape(meta['goal'])}"
    vertex("title", title, "text;html=1;align=left;verticalAlign=top;fontSize=14;whiteSpace=wrap;", X0, 10, width, 50)

    y = Y0
    lane_y = {}
    for key, name, fill, stroke in LANES:
        h = LANE_H[key]
        lane_y[key] = y
        vertex(f"lane_{key}", "", f"html=1;fillColor={fill};strokeColor={stroke};opacity=35;", X0, y, width, h)
        vertex(f"lbl_{key}", f"<b>{escape(name)}</b>", "text;html=1;whiteSpace=wrap;align=left;verticalAlign=middle;",
               X0 + 6, y, LABEL_W, h)
        y += h + 8

    for i, ep in enumerate(eps):
        x = X0 + LABEL_W + 20 + i * COL_W
        status = ep.get("outcome", {}).get("status", "partial")
        o_fill, o_stroke = OUTCOME_COLORS.get(status, OUTCOME_COLORS["partial"])
        sub = " · ".join(escape(ep[k]) for k in ("time", "trigger") if ep.get(k))
        vertex(f"hdr{i}", f"<b>E{i + 1} · {escape(ep.get('title', ''))}</b>"
               + (f"<br><font style='font-size:9px'>{sub}</font>" if sub else ""),
               "text;html=1;align=center;verticalAlign=bottom;whiteSpace=wrap;fontSize=11;", x, Y0 - 58, BOX_W, 56)
        for key, *_ in LANES:
            content = lane_content(ep, key)
            if not content:
                continue
            style = "rounded=1;whiteSpace=wrap;html=1;fontSize=10;align=left;verticalAlign=top;spacing=6;"
            style += (f"fillColor={o_fill};strokeColor={o_stroke};strokeWidth=2;" if key == "outcome"
                      else "fillColor=#ffffff;strokeColor=#999999;")
            vertex(f"c{i}_{key}", content, style, x, lane_y[key] + 5, BOX_W, LANE_H[key] - 10)

    # summary strip under the lanes
    s = trace.get("summary", {})
    blocks = [
        ("Where time went", s.get("time_sinks", [])),
        ("Repeated failure patterns", s.get("failure_patterns", [])),
        ("Missing tools / info that would have helped", s.get("missing_capabilities", [])),
        ("What worked and should be kept", s.get("keep", [])),
    ]
    bw = max(300, (width - 30) // 4)
    for j, (name, items) in enumerate(blocks):
        vertex(f"sum{j}", f"<b>{escape(name)}</b><br>{bullets(items)}",
               "shape=note;whiteSpace=wrap;html=1;align=left;verticalAlign=top;spacing=8;fontSize=11;"
               "fillColor=#fff2cc;strokeColor=#d6b656;size=12;",
               X0 + j * (bw + 10), y + 20, bw, 190)

    page_w, page_h = X0 + width + 40, y + 240
    body = "\n        ".join(cells)
    return f'''<mxfile host="app.diagrams.net">
  <diagram name="Session trace" id="session-trace">
    <mxGraphModel dx="1400" dy="900" grid="1" gridSize="10" guides="1" tooltips="1" connect="1" arrows="1" fold="1" page="1" pageScale="1" pageWidth="{page_w}" pageHeight="{page_h}" math="0" shadow="0">
      <root>
        <mxCell id="0" />
        <mxCell id="1" parent="0" />
        {body}
      </root>
    </mxGraphModel>
  </diagram>
</mxfile>
'''


def main():
    src = sys.argv[1]
    out = sys.argv[2] if len(sys.argv) > 2 else src.rsplit(".", 1)[0] + ".drawio.xml"
    trace = json.load(open(src))
    for n, ep in enumerate(trace["episodes"], 1):
        for field in ("title", "ask", "actions", "outcome"):
            if field not in ep:
                sys.exit(f"episode {n}: missing required field '{field}'")
        if ep["outcome"].get("status") not in OUTCOME_COLORS:
            sys.exit(f"episode {n}: outcome.status must be one of {sorted(OUTCOME_COLORS)}")
    open(out, "w").write(build(trace))
    print(f"wrote {out} ({len(trace['episodes'])} episodes)")


if __name__ == "__main__":
    main()
