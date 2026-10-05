#!/usr/bin/env python3
"""Generates wiring.svg: Raspberry Pi Pico (top view) + 2x TMC2209 StepStick.

    python3 make_wiring.py && convert -density 110 wiring.svg wiring.png

The pin map here must match the constants at the top of apps/pico-owlet/main.cpp.
"""

W, H = 1560, 1500
C = dict(v12="#d62728", gnd="#222222", v33="#e8890c", step="#1f77b4", dir="#2ca02c",
         en="#8a56c2", ms1="#8c564b", ms2="#d6519f", uart="#149aa8", led="#c9a100",
         coil="#888888", off="#aaaaaa")
out = []


def line(pts, col, w=5):
    d = " ".join(f"{x},{y}" for x, y in pts)
    out.append(f'<polyline points="{d}" fill="none" stroke="{col}" stroke-width="{w}" '
               f'stroke-linejoin="round" stroke-linecap="round"/>')


def dot(x, y, col, r=7):
    out.append(f'<circle cx="{x}" cy="{y}" r="{r}" fill="{col}"/>')


def text(x, y, s, size=18, anchor="start", col="#222", weight="normal"):
    out.append(f'<text x="{x}" y="{y}" font-size="{size}" text-anchor="{anchor}" '
               f'fill="{col}" font-weight="{weight}">{s}</text>')


def box(x, y, w, h, fill, stroke="#333", rx=12):
    out.append(f'<rect x="{x}" y="{y}" width="{w}" height="{h}" rx="{rx}" fill="{fill}" '
               f'stroke="{stroke}" stroke-width="3"/>')


# ── Pico, top view, USB at the top ──────────────────────────────
LEFT = ["GP0", "GP1", "GND", "GP2", "GP3", "GP4", "GP5", "GND", "GP6", "GP7",
        "GP8", "GP9", "GND", "GP10", "GP11", "GP12", "GP13", "GND", "GP14", "GP15"]      # pins 1..20
RIGHT = ["VBUS", "VSYS", "GND", "3V3_EN", "3V3", "ADC_VREF", "GP28", "AGND", "GP27", "GP26",
         "RUN", "GP22", "GND", "GP21", "GP20", "GP19", "GP18", "GND", "GP17", "GP16"]    # pins 40..21
PX0, PX1, PY0, PITCH = 300, 560, 230, 36


def ly(pin):        # y of a left-row pin, by pin number 1..20
    return PY0 + (pin - 1) * PITCH


def ry(pin):        # y of a right-row pin, by pin number 40..21
    return PY0 + (40 - pin) * PITCH


# net colour for each used Pico pin
USED_L = {1: "uart", 2: "uart", 4: "step", 5: "dir", 6: "uart", 7: "uart", 9: "dir",
          10: "step", 11: "en", 16: "ms1", 17: "ms2"}
USED_R = {38: "gnd", 36: "v33", 29: "led"}

box(PX0, PY0 - 42, PX1 - PX0, 19 * PITCH + 84, "#f6d5e2")
box(PX0 + 85, PY0 - 78, 90, 56, "#dddddd", rx=10)
text(PX0 + 130, PY0 - 43, "USB-C", 19, "middle", weight="bold")
for i, name in enumerate(LEFT):
    pin, y = i + 1, PY0 + i * PITCH
    col = C[USED_L[pin]] if pin in USED_L else C["off"]
    wt = "bold" if pin in USED_L else "normal"
    dot(PX0, y, col, 6)
    text(PX0 + 14, y + 6, f"{pin:02d}  {name}", 17, "start", col, wt)
for i, name in enumerate(RIGHT):
    pin, y = 40 - i, PY0 + i * PITCH
    col = C[USED_R[pin]] if pin in USED_R else C["off"]
    wt = "bold" if pin in USED_R else "normal"
    dot(PX1, y, col, 6)
    text(PX1 - 14, y + 6, f"{name}  {pin}", 17, "end", col, wt)
text((PX0 + PX1) / 2, PY0 + 19 * PITCH + 70, "Left row: count from USB downward", 15, "middle", "#555")

# ── Drivers, top view (pot side up) ─────────────────────────────
DX0, DX1, DP = 1000, 1240, 38
DL = ["EN", "MS1", "MS2", "PDN", "UART", "CLK", "STEP", "DIR"]
DR = ["VM", "GND", "A2", "A1", "B1", "B2", "VIO", "GND"]


def driver(y0, name):
    box(DX0, y0 - 30, DX1 - DX0, 7 * DP + 60, "#e8e8f5")
    text((DX0 + DX1) / 2, y0 + 128, name, 26, "middle", weight="bold")
    text((DX0 + DX1) / 2, y0 + 154, "TMC2209", 16, "middle")
    text((DX0 + DX1) / 2, y0 + 174, "pot side up", 14, "middle", "#555")
    L, R = {}, {}
    for i in range(8):
        y = y0 + i * DP
        grey = DL[i] in ("UART", "CLK")
        dot(DX0, y, C["off"] if grey else "#333", 6)
        dot(DX1, y, "#333", 6)
        text(DX0 + 14, y + 6, DL[i], 16, "start", C["off"] if grey else "#222")
        text(DX1 - 14, y + 6, DR[i], 16, "end")
        L[DL[i]] = y
        R[f"{DR[i]}{i}"] = y
    return L, R


panL, panR = driver(220, "PAN")
tiltL, tiltR = driver(670, "TILT")

# ── Signal nets: out of the left row, round the bottom, up to the drivers ──
# Outermost first, so the loops nest without crossing on the left and bottom.
NETS = [
    # colour, Pico pins, label,                      driver pins (pan, tilt)
    ("uart", (1, 2),  "GP0 + GP1  →  TILT PDN (UART)", (None, "PDN")),
    ("step", (4,),    "GP2  →  PAN STEP",              ("STEP", None)),
    ("dir",  (5,),    "GP3  →  PAN DIR",               ("DIR", None)),
    ("uart", (6, 7),  "GP4 + GP5  →  PAN PDN (UART)",  ("PDN", None)),
    ("dir",  (9,),    "GP6  →  TILT DIR",              (None, "DIR")),
    ("step", (10,),   "GP7  →  TILT STEP",             (None, "STEP")),
    ("en",   (11,),   "GP8  →  EN (both)",             ("EN", "EN")),
    ("ms1",  (16,),   "GP12  →  MS1 (both)",           ("MS1", "MS1")),
    ("ms2",  (17,),   "GP13  →  MS2 (both)",           ("MS2", "MS2")),
]
N = len(NETS)
for k, (ckey, pins, label, (pan_pin, tilt_pin)) in enumerate(NETS):
    col = C[ckey]
    xl = 60 + k * 26                   # vertical run left of the Pico
    yb = 1050 + (N - 1 - k) * 44       # lane along the bottom
    xr = 892 - k * 26                  # riser left of the drivers
    targets = [y for y in (panL.get(pan_pin), tiltL.get(tilt_pin)) if y is not None]
    top = min(targets)
    line([(PX0, ly(pins[0])), (xl, ly(pins[0])), (xl, yb), (xr, yb), (xr, top), (DX0, top)], col)
    for extra in pins[1:]:             # a second Pico pin on the same net
        line([(PX0, ly(extra)), (xl, ly(extra))], col)
        dot(xl, ly(extra), col)
    for y in targets:
        if y != top:                   # branch to the second driver
            line([(xr, y), (DX0, y)], col)
            dot(xr, y, col)
    text(330, yb - 9, label, 17, "start", col, "bold")

# ── 3.3 V and ground, over the top so they cross no signal ──────
X33, XG, X12 = 1272, 1302, 1440
line([(PX1, ry(38)), (585, ry(38)), (585, 130), (XG, 130), (XG, 1030), (1340, 1030)], C["gnd"])
line([(PX1, ry(36)), (610, ry(36)), (610, 152), (X33, 152), (X33, tiltR["VIO6"])], C["v33"])
text(625, 122, "GND  →  common ground", 16, "start", C["gnd"], "bold")
text(625, 176, "3.3 V  →  VIO", 16, "start", C["v33"], "bold")
for R in (panR, tiltR):
    line([(DX1, R["VIO6"]), (X33, R["VIO6"])], C["v33"]); dot(X33, R["VIO6"], C["v33"])
    for g in ("GND1", "GND7"):
        line([(DX1, R[g]), (XG, R[g])], C["gnd"]); dot(XG, R[g], C["gnd"])
    line([(DX1, R["VM0"]), (X12, R["VM0"])], C["v12"]); dot(X12, R["VM0"], C["v12"])
line([(X12, panR["VM0"]), (X12, 1070), (1370, 1070)], C["v12"])
text(1288, 1036, "Supply −  /  GND", 17, "end", C["gnd"], "bold")
text(1360, 1076, "+12 V supply", 17, "end", C["v12"], "bold")

# ── Motors ──────────────────────────────────────────────────────
for R, name in ((panR, "PAN MOTOR"), (tiltR, "TILT MOTOR")):
    for k in ("A22", "A13", "B14", "B25"):
        line([(DX1, R[k]), (1380, R[k])], C["coil"])
        text(1388, R[k] + 6, k[:2], 16)
        text(1372, R["A22"] - 16, name, 15, "middle", weight="bold")

# ── Activity LED: 3.3 V → resistor → LED → GP22 (the pin sinks it) ──
yl = ry(29)
line([(PX1, yl), (590, yl)], C["led"], 4)
line([(590, yl - 10), (590, yl + 10)], "#111", 3)
out.append(f'<polygon points="608,{yl - 10} 608,{yl + 10} 591,{yl}" fill="{C["led"]}" stroke="#111" stroke-width="1.5"/>')
line([(608, yl), (620, yl)], C["v33"], 4)
out.append(f'<rect x="620" y="{yl - 8}" width="30" height="16" fill="#fff" stroke="#111" stroke-width="2"/>')
line([(650, yl), (664, yl)], C["v33"], 4)
text(600, yl - 18, "LED", 12, "middle", "#555")
text(635, yl - 14, "R", 12, "middle", "#555")
text(640, yl + 28, "3V3", 13, "middle", C["v33"], "bold")

# ── Titles and notes ────────────────────────────────────────────
text(W / 2, 52, "Pico → two TMC2209 drivers", 36, "middle", weight="bold")
text(W / 2, 88, "TOP VIEW  •  components facing you  •  USB at the top", 20, "middle")
nx, ny = 1260, 1160
for i, (s, col, wt) in enumerate([
        ("Crossing wires connect only at a dot.", "#222", "bold"),
        ("12 V goes to VM only, never to the Pico.", "#222", "bold"),
        ("EN is active LOW. The LED pin sinks: low = lit.", "#222", "normal"),
        ("Tilt pair as built: GP6 = DIR, GP7 = STEP.", "#222", "normal"),
        ("PDN (UART): one private line per driver, TX and RX", C["uart"], "normal"),
        ("joined. Sets motor current, reads load and faults.", C["uart"], "normal"),
        ("On-board LED (GP25, no wiring): heartbeat, one flash a second.", "#222", "normal"),
                ("Motor: A1/A2 are one coil, B1/B2 the other.", "#222", "normal"),
        ("Grey driver pins (UART, CLK) stay unconnected.", "#777", "normal"),
        ("Optional: 100 µF 25 V across VM/GND at each driver.", "#777", "normal")]):
    text(nx, ny + i * 28, s, 16, "middle", col, wt)

svg = (f'<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 {W} {H}" width="{W}" height="{H}" '
       f'font-family="DejaVu Sans, Arial, sans-serif"><rect width="{W}" height="{H}" fill="#ffffff"/>'
       + "".join(out) + "</svg>")
open("wiring.svg", "w").write(svg)
print("wrote wiring.svg")
