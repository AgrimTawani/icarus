"""Render the Laya/Icarus architecture diagram as a portable PNG."""

from PIL import Image, ImageDraw, ImageFont

WIDTH, HEIGHT = 1800, 1120
OUT = "docs/assets/laya-decision-architecture-flowchart.png"
FONT = "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf"
BOLD = "/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf"


def font(size, bold=False):
    return ImageFont.truetype(BOLD if bold else FONT, size)


canvas = Image.new("RGB", (WIDTH, HEIGHT), "#f6f9fc")
draw = ImageDraw.Draw(canvas)


def centered(text, y, size, color, bold=False):
    face = font(size, bold)
    left = (WIDTH - draw.textbbox((0, 0), text, font=face)[2]) / 2
    draw.text((left, y), text, font=face, fill=color)


def rounded(box, fill, outline, width=2):
    draw.rounded_rectangle(box, radius=18, fill=fill, outline=outline, width=width)


def box(x, y, w, h, accent, title, lines, tag="DETERMINISTIC", fill="#ffffff"):
    rounded((x, y, x + w, y + h), fill, accent, 3 if tag == "LAYA" else 2)
    tag_w = max(75, len(tag) * 9 + 24)
    draw.rounded_rectangle((x + 20, y + 19, x + 20 + tag_w, y + 46), radius=13,
                           fill=accent)
    draw.text((x + 32, y + 25), tag, font=font(14, True), fill="white")
    title_face = font(22, True)
    tw = draw.textbbox((0, 0), title, font=title_face)[2]
    draw.text((x + (w - tw) / 2, y + 78), title, font=title_face, fill="#102a43")
    for index, line in enumerate(lines):
        line_face = font(17)
        lw = draw.textbbox((0, 0), line, font=line_face)[2]
        draw.text((x + (w - lw) / 2, y + 111 + index * 23), line,
                  font=line_face, fill="#334e68")


def arrow(points, color="#466275", width=4):
    draw.line(points, fill=color, width=width, joint="curve")
    x1, y1 = points[-2]
    x2, y2 = points[-1]
    if abs(x2 - x1) >= abs(y2 - y1):
        direction = 1 if x2 > x1 else -1
        head = [(x2, y2), (x2 - direction * 15, y2 - 8), (x2 - direction * 15, y2 + 8)]
    else:
        direction = 1 if y2 > y1 else -1
        head = [(x2, y2), (x2 - 8, y2 - direction * 15), (x2 + 8, y2 - direction * 15)]
    draw.polygon(head, fill=color)


centered("Icarus: where Laya can make decisions", 22, 39, "#102a43", True)
centered("Laya can be the only learned selector—not the only authority in the flight system.",
         72, 20, "#486581")

draw.text((82, 145), "DETERMINISTIC MISSION AND ACTION CONSTRUCTION", font=font(18, True), fill="#486581")
draw.line((82, 178, 1718, 178), fill="#cbd5e1", width=2)
box(90, 220, 260, 165, "#466275", "Mission request", ["“Inspect, count,", "then return home.”"], "OPERATOR")
box(450, 220, 340, 165, "#2b7bbb", "Objective compiler", ["Parse clauses, order work,", "supply reviewed parameters"], fill="#e9f5ff")
box(890, 220, 365, 165, "#2b7bbb", "Candidate factory", ["Make fewer than 10 complete,", "policy-valid action options"], fill="#e9f5ff")
box(1355, 220, 355, 165, "#d97706", "System-1 selector", ["Choose a candidate /", "return risk or confidence"], "LAYA", "#fff4e5")
arrow([(350, 302), (450, 302)])
arrow([(790, 302), (890, 302)])
arrow([(1255, 302), (1355, 302)])
draw.text((1070, 274), "curated state + candidate IDs", font=font(15), fill="#486581")

draw.text((82, 470), "INDEPENDENT EXECUTION AND SAFETY AUTHORITY", font=font(18, True), fill="#486581")
draw.line((82, 503, 1718, 503), fill="#cbd5e1", width=2)
box(130, 550, 305, 172, "#38804c", "Validate selection", ["Schema, freshness, objective", "order, parameter equality"], fill="#edf7ef")
box(525, 550, 305, 172, "#38804c", "Guardrails", ["State preconditions, geofence,", "limits, collision constraints"], fill="#edf7ef")
box(920, 550, 305, 172, "#38804c", "Mission executor", ["Idempotency, action lifecycle,", "deadline and cancellation"], fill="#edf7ef")
box(1315, 550, 305, 172, "#b42318", "Safety supervisor", ["Hold / RTL / land on faults;", "can override every lower layer"], "HIGHEST AUTHORITY", "#fef0ef")
arrow([(1532, 385), (1532, 470), (282, 470), (282, 550)])
arrow([(435, 636), (525, 636)])
arrow([(830, 636), (920, 636)])
arrow([(1225, 636), (1315, 636)])

rounded((667, 812, 1133, 960), "#ffffff", "#466275", 3)
centered("ArduPilot flight control → aircraft", 850, 22, "#102a43", True)
centered("Only bounded, accepted commands reach the vehicle.", 885, 17, "#334e68")
centered("Pilot override and flight-controller failsafes remain above Icarus.", 916, 15, "#486581")
arrow([(1072, 722), (1072, 770), (900, 770), (900, 812)])
arrow([(1467, 722), (1467, 770), (1133, 770)], "#b42318")
draw.text((1250, 744), "emergency recovery path", font=font(16), fill="#7a271a")

rounded((110, 820, 535, 1025), "#fff7ed", "#f97316", 2)
draw.text((140, 856), "Never delegated to Laya", font=font(19, True), fill="#7a271a")
for i, text in enumerate((
        "• Mission parsing or coverage decisions",
        "• Coordinates, yaw, orbit or other flight parameters",
        "• Arming permission, guardrail approval or failsafes",
        "• MAVLink commands or motor control")):
    draw.text((140, 890 + i * 29), text, font=font(16), fill="#7a271a")

rounded((1260, 820, 1690, 1025), "#fffbeb", "#d97706", 2)
draw.text((1290, 856), "What Laya can own after proof", font=font(19, True), fill="#102a43")
for i, text in enumerate((
        "• Which safe candidate to try next",
        "• Whether to acquire perception or hold",
        "• Recovery-template or macro selection",
        "• Escalation-risk score for human review")):
    draw.text((1290, 890 + i * 29), text, font=font(16), fill="#334e68")

centered("Required before promotion: sealed drone corpus → calibration and safety evaluation → shadow simulation → gated simulator deployment.",
         1065, 15, "#486581")
canvas.save(OUT)
