#!/usr/bin/env python3
"""Proposal diagrams for an event-driven owlet: event-flow.svg and stall-sequence.svg.

    python3 make_event_diagrams.py && for f in event-flow stall-sequence; do convert -density 96 $f.svg $f.png; done
"""

FONT = 'font-family="DejaVu Sans, Arial, sans-serif"'


class Svg:
    def __init__(self, w, h):
        self.w, self.h, self.out = w, h, []

    def box(self, x, y, w, h, fill, title, lines=(), stroke="#333", size=17):
        self.out.append(f'<rect x="{x}" y="{y}" width="{w}" height="{h}" rx="10" fill="{fill}" '
                        f'stroke="{stroke}" stroke-width="2.5"/>')
        ty = y + (h - 20 * len(lines)) / 2 + 6 if lines else y + h / 2 + 6
        self.text(x + w / 2, ty, title, size, "middle", weight="bold")
        for i, s in enumerate(lines):
            self.text(x + w / 2, ty + 22 + 20 * i, s, 14, "middle", "#333")

    def text(self, x, y, s, size=15, anchor="start", col="#222", weight="normal", style=""):
        self.out.append(f'<text x="{x}" y="{y}" font-size="{size}" text-anchor="{anchor}" fill="{col}" '
                        f'font-weight="{weight}" {style}>{s}</text>')

    def arrow(self, pts, col="#333", w=3, label=None, lx=None, ly=None, dash=False):
        d = " ".join(f"{x},{y}" for x, y in pts)
        da = ' stroke-dasharray="8 6"' if dash else ""
        self.out.append(f'<polyline points="{d}" fill="none" stroke="{col}" stroke-width="{w}" '
                        f'stroke-linejoin="round" marker-end="url(#a{col[1:]})"{da}/>')
        self.markers.add(col)
        if label:
            (x1, y1), (x2, y2) = pts[-2], pts[-1]
            self.text(lx if lx is not None else (x1 + x2) / 2, ly if ly is not None else min(y1, y2) - 8,
                      label, 13, "middle", col, "bold")

    markers = set()

    def save(self, path):
        defs = "".join(
            f'<marker id="a{c[1:]}" viewBox="0 0 10 10" refX="9" refY="5" markerWidth="7" markerHeight="7" '
            f'orient="auto-start-reverse"><path d="M0,0 L10,5 L0,10 z" fill="{c}"/></marker>'
            for c in sorted(self.markers))
        open(path, "w").write(
            f'<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 {self.w} {self.h}" width="{self.w}" '
            f'height="{self.h}" {FONT}><defs>{defs}</defs><rect width="{self.w}" height="{self.h}" '
            f'fill="#fff"/>' + "".join(self.out) + "</svg>")
        print("wrote", path)


EV, HW, DRV, SUB, PC, Q = "#c0392b", "#eeeeee", "#dbe9f7", "#dff3df", "#fdebd0", "#fff3bf"

# ── 1. Who tells whom ───────────────────────────────────────────
s = Svg(1640, 1000)
s.text(820, 44, "owlet, event-driven: nobody asks, things announce", 30, "middle", weight="bold")
s.text(820, 74, "Events flow left to right. The only sampling left is at the hardware edge, "
       "inside the driver that owns it.", 16, "middle", "#444")
cols = [("HARDWARE", 40), ("DRIVERS  (event sources)", 330), ("EVENT QUEUE + DISPATCH", 690),
        ("SUBSCRIBERS  (slots)", 1010), ("PC", 1340)]
for name, x in cols:
    s.text(x + 130, 118, name, 15, "middle", "#666", "bold")

rows = [170, 320, 470, 620, 770]
hw = [("Limit switch", ("pin changes",)), ("TMC2209 DIAG pin", ("rises on a stall",)),
      ("TMC2209 registers", ("temperature, shorts,", "load: no interrupt")),
      ("Step generator", ("burst finished",)), ("USB", ("bytes from the PC",))]
drv = [("GPIO interrupt", ("posts LimitHit",)), ("GPIO interrupt", ("posts Stalled",)),
       ("Driver monitor", ("reads on a timer, emits", "ONLY when something changes")),
       ("StepperAxis", ("posts Arrived",)), ("Serial port", ("posts LineReceived",))]
evn = ["LimitHit(axis, end)", "Stalled(axis)", "FaultChanged(axis, flags)", "Arrived", "LineReceived"]
for i, y in enumerate(rows):
    s.box(40, y, 260, 100, HW, *hw[i])
    s.box(330, y, 280, 100, DRV, *drv[i])
    s.arrow([(300, y + 50), (330, y + 50)])
    s.arrow([(610, y + 50), (650, y + 50), (650, 520), (690, 520)], EV)
    s.text(470, y + 124, evn[i], 14, "middle", EV, "bold")
s.box(690, 400, 280, 110, Q, "Event queue", ("fixed size, no heap,", "safe to post from an interrupt"))
s.box(690, 530, 280, 110, Q, "Dispatcher", ("the run loop hands each event", "to whoever connected to it"))
subs = [("Motion policy", ("Stalled / LimitHit / Fault", "→ stop that axis")),
        ("Homing", ("LimitHit → this is zero",)),
        ("Protocol", ("every event → one line", "pushed to the PC: $EVT,…")),
        ("Activity LED", ("Moving / Arrived",))]
sy = [200, 380, 560, 740]
for i, y in enumerate(sy):
    s.box(1010, y, 290, 110, SUB, *subs[i])
    s.arrow([(970, 585), (990, 585), (990, y + 55), (1010, y + 55)], EV)
s.box(1340, 500, 260, 100, PC, "owlet client", ("turns $EVT lines", "into signals"))
s.box(1340, 680, 260, 100, PC, "Your application", ("head.stalled.connect(fn)", "no polling loop"))
s.arrow([(1300, 615), (1320, 615), (1320, 550), (1340, 550)], EV, label="$EVT,STALL,PAN", lx=1470, ly=484)
s.arrow([(1470, 600), (1470, 680)], EV)
s.text(820, 930, "Same seams as today: a new motor driver or sensor only has to post its events; "
       "a new subscriber only has to connect.", 16, "middle", "#333")
s.text(820, 958, "Grey = hardware   blue = code that touches hardware   yellow = the one new piece   "
       "green = existing logic, now reacting   orange = PC side", 14, "middle", "#666")
s.save("event-flow.svg")

# ── 2. One stall, end to end ────────────────────────────────────
q = Svg(1640, 820)
q.text(820, 44, "One stall, end to end: who does what, and in what order", 28, "middle", weight="bold")
q.text(820, 74, "Nobody ever asks “are you stalled?”. The driver chip raises a pin and the news travels.",
       16, "middle", "#444")
life = ["Motor + TMC2209", "Pin interrupt", "Event queue", "Run loop", "Motion policy", "Protocol",
        "owlet client (PC)", "Your app"]
xs = [110 + i * 203 for i in range(len(life))]
fill = [HW, DRV, Q, Q, SUB, SUB, PC, PC]
for x, name, f in zip(xs, life, fill):
    q.box(x - 88, 110, 176, 54, f, name, size=15)
    q.out.append(f'<line x1="{x}" y1="164" x2="{x}" y2="780" stroke="#999" stroke-width="2" '
                 f'stroke-dasharray="6 6"/>')
steps = [
    (0, 0, "1  shaft blocked: load crosses the stall threshold", None),
    (0, 1, "2  DIAG pin goes high", EV),
    (1, 2, "3  post Stalled(pan)   (a few microseconds, then the interrupt returns)", EV),
    (2, 3, "4  next loop pass takes it off the queue", EV),
    (3, 4, "5  deliver to subscribers", EV),
    (4, 4, "6  stop the pan axis, keep the true step count", None),
    (3, 5, "7  deliver to subscribers", EV),
    (5, 6, "8  $EVT,STALL,PAN   (pushed, not requested)", EV),
    (6, 7, "9  stalled signal fires your handler", EV),
]
y = 215
for a, b, label, col in steps:
    if a == b:
        q.out.append(f'<rect x="{xs[a] - 9}" y="{y - 20}" width="18" height="34" fill="#666"/>')
        q.text(xs[a] + 18, y + 2, label, 14, "start", "#222")
    else:
        q.arrow([(xs[a], y), (xs[b], y)], col, 3)
        q.text((xs[a] + xs[b]) / 2 if b - a > 1 else xs[a] + 12, y - 10, label, 14,
               "middle" if b - a > 1 else "start", col, "bold")
    y += 62
q.text(820, 800, "Steps 2 to 8 take about one loop pass (2 ms). The same path carries limit switches, "
       "driver faults and arrivals.", 15, "middle", "#333")
q.save("stall-sequence.svg")
