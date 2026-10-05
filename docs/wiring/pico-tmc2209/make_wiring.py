#!/usr/bin/env python3
"""Generates wiring.svg: Raspberry Pi Pico + 2x TMC2209 StepStick + 2x NEMA 17."""

W, H = 1240, 985
C = dict(v12="#d62728", gnd="#111111", v33="#ff8c00", led="#e6b800", step="#1f77b4", ms1="#8c564b", ms2="#e377c2",
         dir="#2ca02c", en="#9467bd", coil="#7f7f7f", nc="#aaaaaa")
out = []


def line(pts, col, w=3):
    d = " ".join(f"{x},{y}" for x, y in pts)
    out.append(f'<polyline points="{d}" fill="none" stroke="{col}" stroke-width="{w}" '
               f'stroke-linejoin="round" stroke-linecap="round"/>')


def dot(x, y, col):
    out.append(f'<circle cx="{x}" cy="{y}" r="5.5" fill="{col}"/>')


def text(x, y, s, size=14, anchor="start", col="#111", weight="normal"):
    out.append(f'<text x="{x}" y="{y}" font-size="{size}" text-anchor="{anchor}" '
               f'fill="{col}" font-weight="{weight}">{s}</text>')


def box(x, y, w, h, fill, stroke="#333", rx=8):
    out.append(f'<rect x="{x}" y="{y}" width="{w}" height="{h}" rx="{rx}" fill="{fill}" '
               f'stroke="{stroke}" stroke-width="2"/>')


# ── Pico ────────────────────────────────────────────────────────
PX0, PX1 = 50, 260
box(PX0, 140, PX1 - PX0, 620, "#f7d6e3")
text(155, 172, "Raspberry Pi Pico", 17, "middle", weight="bold")
text(155, 192, "(USB-C clone, RP2040)", 13, "middle")
box(115, 112, 80, 30, "#cccccc", rx=4)
text(155, 132, "USB to PC", 12, "middle")
pico = {"3V3": (230, "3V3(OUT)  pin 36"), "GP2": (336, "GP2  pin 4"), "GP3": (372, "GP3  pin 5"),
        "GP12": (446, "GP12  pin 16"), "GP13": (464, "GP13  pin 17"),
        "GP8": (500, "GP8  pin 11"), "GP6": (600, "GP6  pin 9"), "GP7": (636, "GP7  pin 10"),
        "GP22": (678, "GP22  pin 29"),
        "GND": (720, "GND  pin 38")}
for k, (y, label) in pico.items():
    text(PX1 - 10, y + 5, label, 14, "end")
    out.append(f'<circle cx="{PX1}" cy="{y}" r="4" fill="#333"/>')

# ── Drivers (top view: potentiometer / heatsink side up) ────────
DX0, DX1 = 560, 760
LEFT = ["EN", "MS1", "MS2", "PDN", "UART", "CLK", "STEP", "DIR"]
RIGHT = ["VM", "GND", "A2", "A1", "B1", "B2", "VIO", "GND"]


def driver(y0, name):
    box(DX0, y0 - 30, DX1 - DX0, 312, "#e8e8f5")
    text(660, y0 + 118, name, 16, "middle", weight="bold")
    text(660, y0 + 138, "TMC2209", 13, "middle")
    text(660, y0 + 156, "top view", 12, "middle", "#555")
    text(660, y0 + 172, "(pot side up)", 12, "middle", "#555")
    ys = [y0 + 36 * i for i in range(8)]
    for i, y in enumerate(ys):
        lc = C["nc"] if LEFT[i] in ("PDN", "UART", "CLK") else "#111"
        text(DX0 + 10, y + 5, LEFT[i], 13, "start", lc)
        text(DX1 - 10, y + 5, RIGHT[i], 13, "end")
        out.append(f'<circle cx="{DX0}" cy="{y}" r="4" fill="#333"/>')
        out.append(f'<circle cx="{DX1}" cy="{y}" r="4" fill="#333"/>')
    return dict(zip(LEFT, ys)), dict(zip(["VM", "GND1", "A2", "A1", "B1", "B2", "VIO", "GND2"], ys))


panL, panR = driver(120, "PAN driver")
tiltL, tiltR = driver(530, "TILT driver")

X12, XG, X33, XEN, XVIO = 810, 880, 470, 330, 785

# ── Power: 12 V and ground buses ────────────────────────────────
line([(X12, 92), (X12, tiltR["VM"])], C["v12"], 4)
line([(XG, 92), (XG, 890), (300, 890), (300, pico["GND"][0]), (PX1, pico["GND"][0])], C["gnd"], 4)
for R in (panR, tiltR):
    line([(DX1, R["VM"]), (X12, R["VM"])], C["v12"]); dot(X12, R["VM"], C["v12"])
    for g in ("GND1", "GND2"):
        line([(DX1, R[g]), (XG, R[g])], C["gnd"]); dot(XG, R[g], C["gnd"])
# capacitors across each driver's VM/GND
for y in (92, 500):
    line([(X12, y), (838, y)], C["v12"]); line([(852, y), (XG, y)], C["gnd"])
    line([(838, y - 12), (838, y + 12)], "#111", 3); line([(852, y - 12), (852, y + 12)], "#111", 3)
    text(832, y - 16, "+", 15, "end", C["v12"], "bold")
    dot(X12, y, C["v12"]); dot(XG, y, C["gnd"])
text(895, 72, "100 µF 25 V at each driver.", 13)
text(895, 89, "Optional if the supply has a large", 13)
text(895, 106, "output capacitor and short wires.", 13)
# supply
box(985, 425, 200, 80, "#fde9c9")
text(1085, 452, "12 V DC supply", 15, "middle", weight="bold")
text(1085, 472, "USB-C PD trigger board", 13, "middle")
text(1085, 490, "set to 12 V, short wires", 13, "middle")
line([(985, 445), (X12, 445)], C["v12"], 4); dot(X12, 445, C["v12"])
line([(985, 485), (XG, 485)], C["gnd"], 4); dot(XG, 485, C["gnd"])
text(975, 439, "+", 16, "end", C["v12"], "bold"); text(975, 503, "−", 16, "end", C["gnd"], "bold")

# ── 3.3 V logic supply ──────────────────────────────────────────
line([(PX1, pico["3V3"][0]), (X33, pico["3V3"][0])], C["v33"])
line([(X33, pico["3V3"][0]), (X33, 870), (XVIO, 870), (XVIO, tiltR["VIO"]), (DX1, tiltR["VIO"])], C["v33"])
line([(X33, 420), (XVIO, 420), (XVIO, panR["VIO"]), (DX1, panR["VIO"])], C["v33"]); dot(X33, 420, C["v33"])

# ── Microstep select, shared by both drivers ────────────────────
for key, gp, xt in (("MS1", "GP12", 500), ("MS2", "GP13", 530)):
    col = C[key.lower()]
    line([(PX1, pico[gp][0]), (xt, pico[gp][0])], col)
    line([(DX0, panL[key]), (xt, panL[key]), (xt, tiltL[key]), (DX0, tiltL[key])], col)
    dot(xt, pico[gp][0], col)

# ── Signals ─────────────────────────────────────────────────────
line([(PX1, pico["GP2"][0]), (DX0, panL["STEP"])], C["step"])
line([(PX1, pico["GP3"][0]), (DX0, panL["DIR"])], C["dir"])
# As built, the tilt pair is the other way round from the pan pair:
# GP6 is DIR and GP7 is STEP.
line([(PX1, pico["GP6"][0]), (400, pico["GP6"][0]), (400, tiltL["DIR"]), (DX0, tiltL["DIR"])], C["dir"])
line([(PX1, pico["GP7"][0]), (370, pico["GP7"][0]), (370, tiltL["STEP"]), (DX0, tiltL["STEP"])], C["step"])
yen = pico["GP8"][0]
line([(PX1, yen), (XEN, yen)], C["en"])
line([(DX0, panL["EN"]), (XEN, panL["EN"]), (XEN, tiltL["EN"]), (DX0, tiltL["EN"])], C["en"])
dot(XEN, yen, C["en"])
# EN pull-up
line([(XEN, yen), (372, yen)], C["en"]); line([(428, yen), (X33, yen)], C["v33"]); dot(X33, yen, C["v33"])
out.append(f'<rect x="372" y="{yen - 9}" width="56" height="18" fill="#fff" stroke="#111" stroke-width="2"/>')
text(400, yen + 5, "10 kΩ", 12, "middle")
text(400, yen + 26, "optional pull-up", 12, "middle", "#555")

# ── Activity LED (3.3 V → resistor → LED → GP22; the pin sinks it) ──
yl = pico["GP22"][0]
# In the order it is built: 3.3 V, resistor, LED anode, LED cathode, pin.
line([(PX1, yl), (273, yl)], C["led"])
line([(273, yl - 9), (273, yl + 9)], "#111", 2.5)  # cathode bar, towards the pin
out.append(f'<polygon points="290,{yl - 9} 290,{yl + 9} 274,{yl}" fill="{C["led"]}" stroke="#111" stroke-width="1.5"/>')
line([(290, yl), (300, yl)], C["v33"])
out.append(f'<rect x="300" y="{yl - 7}" width="28" height="14" fill="#fff" stroke="#111" stroke-width="2"/>')
line([(328, yl), (344, yl), (344, yl + 8)], C["v33"])
text(344, yl + 22, "3V3", 12, "middle", C["v33"], "bold")
text(281, yl - 14, "LED", 11, "middle", "#555")
text(314, yl - 12, "R", 11, "middle", "#555")

# ── Motors ──────────────────────────────────────────────────────
def motor(R, name):
    y0 = R["A2"] - 26
    box(985, y0, 170, 160, "#dddddd")
    text(1070, y0 + 150, name, 14, "middle", weight="bold")
    for k in ("A2", "A1", "B1", "B2"):
        line([(DX1, R[k]), (985, R[k])], C["coil"])
    text(995, R["A2"] + 23, "coil 1 (one wire pair)", 13)
    text(995, R["B1"] + 23, "coil 2 (other pair)", 13)


motor(panR, "PAN motor (in base)")
motor(tiltR, "TILT motor (on yoke)")

# ── Labels and legend ───────────────────────────────────────────
text(W / 2, 40, "Pan-tilt wiring: Pico + 2 × TMC2209 + 2 × NEMA 17", 22, "middle", weight="bold")
text(W / 2, 62, "Wires join only where there is a dot. Grey driver pins (PDN, UART, CLK) stay unconnected.", 13,
     "middle", "#444")
lx, ly = 50, 780
box(lx, ly, 262, 200, "#ffffff", "#999", 6)
for i, (k, lab) in enumerate([("v12", "12 V motor power"), ("gnd", "Ground (all joined)"),
                              ("v33", "3.3 V logic from Pico"), ("step", "STEP"), ("dir", "DIR"),
                              ("ms1", "MS1 (shared)"), ("ms2", "MS2 (shared)"),
                              ("en", "ENABLE (shared, active low)"), ("led", "Activity LED (pin sinks: low = lit)"),
                              ("coil", "Motor coil wires")]):
    y = ly + 18 + 18 * i
    line([(lx + 12, y), (lx + 46, y)], C[k], 4)
    text(lx + 56, y + 5, lab, 13)

svg = (f'<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 {W} {H}" width="{W}" height="{H}" '
       f'font-family="DejaVu Sans, Arial, sans-serif"><rect width="{W}" height="{H}" fill="#ffffff"/>'
       + "".join(out) + "</svg>")
open("wiring.svg", "w").write(svg)
print("wrote wiring.svg")
