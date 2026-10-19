#!/usr/bin/env python3
"""Render the reviewed Icarus V2 prompt-to-motors architecture diagram."""

from itertools import pairwise
from pathlib import Path
from textwrap import wrap

from PIL import Image, ImageDraw, ImageFont

ROOT = Path(__file__).resolve().parent
OUT = ROOT / "icarus-v2-dcm-to-motors-architecture.png"
W, H = 2800, 1840

COLORS = {
    "ink": "#172B4D",
    "muted": "#53657A",
    "blue": "#0969C7",
    "blue_fill": "#DCEEFF",
    "gold": "#C58A00",
    "gold_fill": "#FFF5D6",
    "orange": "#E76518",
    "orange_fill": "#FFE7D3",
    "purple": "#7B2CBF",
    "purple_fill": "#EEDCFA",
    "green": "#2E8B42",
    "green_fill": "#E5F5E6",
    "red": "#D82632",
    "red_fill": "#FFE2E4",
    "gray": "#667085",
    "gray_fill": "#F4F6F8",
    "white": "#FFFFFF",
}

FONT = "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf"
FONT_BOLD = "/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf"


def font(size, bold=False):
    return ImageFont.truetype(FONT_BOLD if bold else FONT, size)


img = Image.new("RGB", (W, H), COLORS["white"])
d = ImageDraw.Draw(img)


def rounded_box(xy, fill, outline, radius=18, width=4):
    d.rounded_rectangle(xy, radius=radius, fill=fill, outline=outline, width=width)


def dashed_rect(xy, color, width=4, dash=16, gap=10, radius=0):
    x1, y1, x2, y2 = xy
    if radius:
        # A subtle solid rounded frame plus white gaps gives a stable dashed look.
        d.rounded_rectangle(xy, radius=radius, outline=color, width=width)
    else:
        for x in range(x1, x2, dash + gap):
            d.line((x, y1, min(x + dash, x2), y1), fill=color, width=width)
            d.line((x, y2, min(x + dash, x2), y2), fill=color, width=width)
        for y in range(y1, y2, dash + gap):
            d.line((x1, y, x1, min(y + dash, y2)), fill=color, width=width)
            d.line((x2, y, x2, min(y + dash, y2)), fill=color, width=width)


def centered_text(xy, title, lines=(), title_size=23, body_size=16,
                  title_color=None, body_color=None, max_chars=40):
    x1, y1, x2, y2 = xy
    title_color = title_color or COLORS["ink"]
    body_color = body_color or COLORS["muted"]
    tf = font(title_size, True)
    bf = font(body_size)
    title_lines = []
    for part in title.split("\n"):
        title_lines.extend(wrap(part, max_chars) or [""])
    body_lines = []
    for item in lines:
        body_lines.extend(wrap(item, max_chars) or [""])
    heights = [d.textbbox((0, 0), s, font=tf)[3] for s in title_lines]
    heights += [d.textbbox((0, 0), s, font=bf)[3] for s in body_lines]
    total = sum(heights) + 7 * max(0, len(heights) - 1)
    y = y1 + (y2 - y1 - total) / 2
    for s in title_lines:
        box = d.textbbox((0, 0), s, font=tf)
        d.text(((x1 + x2 - (box[2] - box[0])) / 2, y), s, font=tf, fill=title_color)
        y += box[3] - box[1] + 7
    for s in body_lines:
        box = d.textbbox((0, 0), s, font=bf)
        d.text(((x1 + x2 - (box[2] - box[0])) / 2, y), s, font=bf, fill=body_color)
        y += box[3] - box[1] + 7


def box(xy, title, lines, kind="blue", max_chars=40, title_size=23, body_size=16):
    rounded_box(xy, COLORS[f"{kind}_fill"], COLORS[kind])
    centered_text(xy, title, lines, title_size, body_size, max_chars=max_chars)


def arrow(points, color="blue", width=5, dashed=False, head=16):
    fill = COLORS.get(color, color)
    if dashed:
        for a, b in pairwise(points):
            x1, y1 = a
            x2, y2 = b
            length = max(abs(x2 - x1), abs(y2 - y1))
            steps = max(1, int(length / 22))
            for i in range(0, steps, 2):
                t1, t2 = i / steps, min((i + 1) / steps, 1)
                d.line((x1 + (x2 - x1) * t1, y1 + (y2 - y1) * t1,
                        x1 + (x2 - x1) * t2, y1 + (y2 - y1) * t2),
                       fill=fill, width=width)
    else:
        d.line(points, fill=fill, width=width, joint="curve")
    x1, y1 = points[-2]
    x2, y2 = points[-1]
    if abs(x2 - x1) >= abs(y2 - y1):
        direction = 1 if x2 > x1 else -1
        tri = [(x2, y2), (x2 - direction * head, y2 - head * 0.65),
               (x2 - direction * head, y2 + head * 0.65)]
    else:
        direction = 1 if y2 > y1 else -1
        tri = [(x2, y2), (x2 - head * 0.65, y2 - direction * head),
               (x2 + head * 0.65, y2 - direction * head)]
    d.polygon(tri, fill=fill)


def layer_label(number, label, y1, y2, color):
    cy = (y1 + y2) // 2
    d.line((220, y1 + 8, 220, y2 - 8), fill=COLORS[color], width=8)
    d.line((220, cy, 267, cy), fill=COLORS[color], width=7)
    d.ellipse((238, cy - 24, 286, cy + 24), fill=COLORS[color])
    n = str(number)
    bb = d.textbbox((0, 0), n, font=font(20, True))
    d.text((262 - (bb[2] - bb[0]) / 2, cy - (bb[3] - bb[1]) / 2 - 2),
           n, font=font(20, True), fill=COLORS["white"])
    parts = label.split("\n")
    ty = cy - len(parts) * 17
    for part in parts:
        bb = d.textbbox((0, 0), part, font=font(25, True))
        d.text((205 - (bb[2] - bb[0]), ty), part, font=font(25, True),
               fill=COLORS[color])
        ty += 34


# ------------------------------------------------------------------ heading
title = "Icarus V2 Phase 1 — Prompt-to-Motors Architecture"
bb = d.textbbox((0, 0), title, font=font(49, True))
d.text(((W - (bb[2] - bb[0])) / 2, 30), title, font=font(49, True), fill=COLORS["ink"])
subtitle = "One semantic mission plan • preflight visual schedule • minimal deterministic sequencing • independent safety authority"
bb = d.textbbox((0, 0), subtitle, font=font(19))
d.text(((W - (bb[2] - bb[0])) / 2, 94), subtitle, font=font(19), fill=COLORS["muted"])

# Legend
legend = [("blue", "Prompt / data"), ("orange", "Authoritative command"),
          ("green", "State / result feedback"), ("red", "Safety override")]
lx, ly = 1570, 132
for color, label in legend:
    arrow([(lx, ly + 10), (lx + 62, ly + 10)], color=color, width=4, head=12)
    d.text((lx + 76, ly - 2), label, font=font(15), fill=COLORS["ink"])
    lx += 300

# --------------------------------------------------------------- layer frames
frames = [
    (1, "OPERATOR\nLAYER", 175, 330, "blue", 2290),
    (2, "PROMPT &\nCOGNITION", 350, 930, "gold", 2765),
    (3, "AUTHORITY &\nSAFETY", 950, 1315, "orange", 2290),
    (4, "CONTROL\nLAYER", 1335, 1495, "purple", 2290),
    (5, "DEVICE /\nSIMULATION", 1515, 1745, "purple", 2290),
]
for n, label, y1, y2, color, x2 in frames:
    layer_label(n, label, y1, y2, color)
    dashed_rect((300, y1, x2, y2), COLORS[color], width=3, dash=15, gap=10)

# -------------------------------------------------------------- operator row
box((340, 205, 770, 305), "Natural-language mission", ["Operator text is preserved verbatim"], "blue", 42)
box((900, 205, 1390, 305), "dcm-fly + control lease", ["CLI / SDK session, authority request", "V3 context is the default"], "blue", 42)
box((1510, 205, 2225, 305), "Validated mission + visual plan", ["Ready • clarification • unsupported", "Objectives and YOLO/VLM calls fixed before flight"], "blue", 56)
arrow([(770, 255), (900, 255)], "blue")
arrow([(1390, 255), (1510, 255)], "blue")

# ------------------------------------------------------ prompt source assembly
d.text((335, 372), "SEMANTIC NLP + DETERMINISTIC CONTEXT", font=font(20, True), fill=COLORS["gold"])
source_boxes = [
    ((340, 415, 690, 520), "LLM semantic planner", ["Intent + safe obvious defaults", "ready / clarify / unsupported"]),
    ((340, 540, 690, 645), "Narrow plan validator", ["Schema • bounds • provenance", "no regex plan or semantic rewrite"]),
    ((340, 665, 690, 770), "Visual-plan compiler", ["Exact YOLO/VLM call budget", "trigger + capture window"]),
    ((340, 790, 690, 895), "Compiled objective plan", ["Ordered typed actions", "one result unlocks the next"]),
]
for xy, t, lines in source_boxes:
    box(xy, t, lines, "gold", 31, 20, 14)

profile_boxes = [
    ((725, 415, 1050, 500), "EXACT OPERATOR TEXT", ["Preserved verbatim", "one semantic authority"]),
    ((725, 515, 1050, 600), "SOUL + LIVE BODY", ["3 compact principles", "airframe limits, no world facts"]),
    ((725, 615, 1050, 700), "ENVIRONMENT CONTEXT", ["Always blank at planning", "only live perception reveals the world"]),
    ((725, 715, 1050, 800), "REVIEWED DEFAULT POLICY", ["Lifecycle prerequisites", "bounded safe defaults only"]),
    ((725, 815, 1050, 895), "CAPABILITY CONTRACT", ["Actions + argument bounds", "YOLO/VLM availability"]),
]
for xy, t, lines in profile_boxes:
    box(xy, t, lines, "gold", 30, 19, 14)

box((1100, 445, 1435, 865), "Preflight planning input", [
    "Allow-list fields only", "Exact user prompt",
    "Reviewed defaults + limits", "Empty environment context",
    "No scenario landmarks or truth",
    "One LLM call before flight", "Schema-constrained output",
    "One bounded repair attempt",
    "No per-action LLM context"], "gold", 31, 23, 16)

# converging source arrows
for y in (468, 592, 717, 842):
    arrow([(690, y), (1090, y)], "gold", width=4, head=13)
for y in (457, 557, 657, 757, 855):
    arrow([(1050, y), (1090, y)], "gold", width=4, head=13)

# --------------------------------------------------------- actual prompt block
rounded_box((1485, 390, 2245, 900), COLORS["blue_fill"], COLORS["blue"], 22, 5)
d.text((1515, 410), "ONE PREFLIGHT SEMANTIC-PLANNING PROMPT", font=font(22, True), fill=COLORS["blue"])
box((1515, 458, 2215, 655), "SYSTEM message", [
    "Semantic mission-planner role; no flight authority",
    "typed objectives + argument/provenance rules",
    "safe defaults + clarification boundary",
    "YOLO/VLM scheduling rules",
    "never invent targets, coordinates or world facts"], "blue", 61, 21, 15)
box((1515, 685, 2215, 865), "USER message: Plan this mission", [
    "exact operator text",
    "reviewed policy + limited live flight state",
    "motion limits + perception capability",
    "environment_context = empty",
    "Return one structured mission object"], "blue", 61, 21, 15)
arrow([(1435, 655), (1485, 655)], "blue")

# ----------------------------------------------------- DCM and validation row
box((2325, 390, 2745, 550), "Local Qwen DCM", ["one semantic planning inference", "fills only reviewed safe gaps", "asks when mission meaning is ambiguous"], "blue", 35, 24, 16)
box((2325, 590, 2745, 720), "Typed mission JSON", ["status • objectives • arguments", "provenance • defaults • clarification"], "blue", 38, 22, 15)
box((2325, 760, 2745, 900), "Plan + visual compiler", ["grounding • bounds • capability • safety", "exact YOLO/VLM triggers and maximum calls"], "blue", 39, 22, 15)
arrow([(2245, 500), (2325, 500)], "blue")
arrow([(2535, 550), (2535, 590)], "blue")
arrow([(2535, 720), (2535, 760)], "blue")

# ------------------------------------------------------- authority/control row
box((340, 990, 700, 1095), "Whole-plan approval", ["Objectives + visual budget", "or autonomous simulation"], "orange", 32)
box((790, 990, 1160, 1095), "Drone API", ["gRPC + Protobuf", "typed command boundary"], "orange", 30)
box((1250, 990, 1690, 1095), "Lease + C++ guardrails", ["Authority, mode, geofence", "limits and failsafes"], "orange", 39)
box((1780, 990, 2250, 1095), "Minimal plan executor", ["next validated step or stop", "no NLP and no per-action LLM call"], "orange", 42)
arrow([(2535, 900), (2535, 960), (520, 960), (520, 990)], "blue")
arrow([(700, 1042), (790, 1042)], "orange")
arrow([(1160, 1042), (1250, 1042)], "orange")
arrow([(1690, 1042), (1780, 1042)], "orange")

# Vision side path within authority layer
box((340, 1145, 825, 1268), "Semantic perception boundary", ["detect → YOLO person counts", "inspect_scene → CPU Qwen2-VL description", "camera evidence has no flight authority"], "green", 42, 21, 14)
box((910, 1145, 1395, 1268), "Result → plan progress", ["Typed success/failure + evidence", "next objective unlocks only on success"], "green", 42, 21, 14)
arrow([(2015, 1095), (2015, 1118), (582, 1118), (582, 1145)], "green", width=4)
arrow([(825, 1207), (910, 1207)], "green", width=4)
arrow([(1395, 1207), (1470, 1207), (1470, 1120), (2015, 1120), (2015, 1095)], "green", width=4, dashed=True)

# Safety supervisor
box((1515, 1145, 2250, 1278), "Independent safety supervisor", [
    "Monitors state, geofence, leases and DCM heartbeat",
    "May BRAKE → RTL → LAND without model permission"], "red", 63, 23, 16)
arrow([(1880, 1145), (1880, 1095)], "red", width=5, dashed=True)

# -------------------------------------------------------------- control layer
box((340, 1365, 1100, 1465), "MAVLink gateway", ["Command encoding • telemetry • mode control"], "purple", 60)
box((1240, 1365, 2250, 1465), "ArduPilot flight stack", ["Guidance → attitude/rate control → motor mixer", "LLM never controls attitude, rates, ESCs or motors"], "purple", 78)
arrow([(2015, 1095), (2015, 1328), (720, 1328), (720, 1365)], "orange")
arrow([(1100, 1415), (1240, 1415)], "orange")

# --------------------------------------------------------------- device row
box((340, 1555, 930, 1705), "Gazebo baseline", ["ArduPilotPlugin → MotorBridge", "custom Akshu airframe + rotor physics", "promotion baseline"], "purple", 48, 23, 16)
box((1010, 1555, 1600, 1705), "AirSim realism track", ["ArduPilot AirSim backend", "default AirSim multirotor physics", "RGB / depth / LiDAR • not airframe-equivalent"], "purple", 50, 23, 16)
box((1680, 1555, 2250, 1705), "Physical aircraft (future)", ["Pixhawk → ESCs → motors", "requires independent hardware validation"], "purple", 48, 23, 16)
for x in (635, 1305, 1965):
    arrow([(1745, 1465), (1745, 1518), (x, 1518), (x, 1555)], "orange")

# ----------------------------------------------------------- feedback lane
dashed_rect((2320, 950, 2745, 1745), COLORS["green"], width=3, dash=15, gap=10)
d.text((2370, 970), "FEEDBACK & LEARNING", font=font(22, True), fill=COLORS["green"])
box((2350, 1020, 2715, 1140), "Vehicle feedback", ["Telemetry • perception", "action status • errors"], "green", 31, 21, 15)
box((2350, 1180, 2715, 1300), "Sealed episode", ["Prompt • plan • mission steps", "state • results • media"], "green", 31, 21, 15)
box((2350, 1340, 2715, 1460), "Offline aggregation", ["Reliability • regression", "safety and performance"], "green", 31, 21, 15)
box((2350, 1500, 2715, 1630), "Human review + promotion", ["Proposals cannot self-promote", "no path back into live control"], "green", 31, 21, 15)
arrow([(2532, 1140), (2532, 1180)], "green", dashed=True)
arrow([(2532, 1300), (2532, 1340)], "green")
arrow([(2532, 1460), (2532, 1500)], "green")
arrow([(2250, 1415), (2300, 1415), (2300, 1080), (2350, 1080)], "green", width=4, dashed=True)
d.text((2380, 1662), "Offline only — never writes live policy", font=font(17, True), fill=COLORS["green"])

# Footer notes
d.text((330, 1778), "The LLM understands intent once; deterministic checks enforce schema, provenance, bounds, capability, sequencing and execution safety.",
       font=font(18, True), fill=COLORS["ink"])
d.text((330, 1810), "No per-action LLM loop: only the validated plan crosses the Drone API one step at a time; safety software and ArduPilot retain final authority.",
       font=font(18), fill=COLORS["muted"])

img.save(OUT, optimize=True)
print(OUT)
