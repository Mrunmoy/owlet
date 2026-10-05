# Pan-tilt wiring: Pico + 2 x TMC2209 + 2 x NEMA 17

Diagram: `wiring.png` (or `wiring.svg`). Regenerate with `python3 make_wiring.py`, then `convert -density 110 wiring.svg wiring.png`.

The firmware for this wiring is `apps/pico-owlet` (`./build.py --pico --flash`). The pin numbers
below are the constants at the top of `apps/pico-owlet/main.cpp`; change both together.

## Driver pinout (top view, potentiometer side up, pins down)

Matches the three labels printed on the board: EN top-left, DIR bottom-left, GND bottom-right.
**Assumed** to be the standard StepStick TMC2209 layout; check it against the seller's pinout picture.

| Left column | Right column |
|-------------|--------------|
| EN          | VM (12 V)    |
| MS1         | GND          |
| MS2         | A2           |
| PDN         | A1           |
| UART        | B1           |
| CLK         | B2           |
| STEP        | VIO (3.3 V)  |
| DIR         | GND          |

## Connections

| From                     | To                                  | Notes |
|--------------------------|-------------------------------------|-------|
| Pico GP2 (pin 4)         | Pan driver STEP                     | |
| Pico GP3 (pin 5)         | Pan driver DIR                      | |
| Pico GP6 (pin 9)         | Tilt driver DIR                     | The tilt pair is the other way round from the pan pair. |
| Pico GP7 (pin 10)        | Tilt driver STEP                    | |
| Pico GP8 (pin 11)        | EN on both drivers                  | Active low. Optional 10 k pull-up to 3.3 V keeps the motors off until firmware enables them. |
| Pico GP12 (pin 16)       | MS1 on both drivers                 | Microstep select, shared. No resistor needed. |
| Pico GP13 (pin 17)       | MS2 on both drivers                 | Microstep select, shared. No resistor needed. |
| Pico 3V3(OUT) (pin 36)   | VIO on both drivers                 | Logic supply only. |
| Pico GP22 (pin 29)       | Activity LED cathode                | 3.3 V → resistor → LED → GP22: the pin sinks the LED (low = lit). Lit while the head is moving. Use about 330 Ω to 1 kΩ; with 10 kΩ it is barely visible. |
| Pico GND (pin 38)        | Driver GND pins and supply negative | Every ground joined together. |
| 12 V supply positive     | VM on both drivers                  | Never to the Pico. |
| 100 uF 25 V capacitor    | Across VM and GND at each driver    | Stripe (negative) to GND. Optional when the supply has a large output capacitor and wires under about 15 cm. |
| Motor coil 1             | A2 and A1                           | |
| Motor coil 2             | B1 and B2                           | |

PDN, UART and CLK stay unconnected. The Pico is powered from USB only.

## Microstepping

| GP13 (MS2) | GP12 (MS1) | Microsteps |
|------------|------------|------------|
| low        | low        | 1/8        |
| low        | high       | 1/32       |
| high       | low        | 1/64       |
| high       | high       | 1/16       |

The driver has internal pull-downs on MS1 and MS2, so it sits at 1/8 until the firmware sets the pins.

At 1/16 with the 3:1 belt: 200 x 16 x 3 = 9600 steps per output revolution (0.0375 degrees per step).

## Finding the motor coil pairs

Use a multimeter on resistance. Two wires that read a few ohms between them are one coil.
The other two are the second coil. Swapping the two wires of one coil reverses the direction.

## Power supply

A USB-C PD trigger board set to 12 V, with its 1000 uF output capacitor, is the intended supply.
Set the switches before connecting anything, and confirm 12 V with a multimeter first.
9 V or 15 V also work; avoid 20 V and above without capacitors at the drivers.

The voltage printed on a stepper (4.1 V on these) is its rated current times its coil
resistance, not the supply to use: the driver regulates current, and a higher supply is
what keeps torque up at speed.

**As run (2026-10-05):** first on a 5 V 5 A supply (5.19 V measured), which is just above the
TMC2209's 4.75 V minimum and was enough for slow moves. Then on a 12 V 2 A adapter (13.3 V
measured): 0.23 A with both motors holding, 0.81 A peak with both moving hard, motors and
drivers cool. The driver potentiometers are as shipped. With heatsinks fitted they are hard to
reach; the supply current is an easy way to check the setting without touching them.

## Bring-up order

1. Wire everything with the motors unplugged and the 12 V supply off.
2. Switch on 12 V. Set each driver's current limit with the potentiometer: measure the voltage
   between the pot's metal top and GND, and start at about 0.6 V. Clockwise/anticlockwise sense
   varies between boards, so turn a little and re-measure.
3. Switch 12 V off. Plug in the motors. Stick the heatsinks on the copper pad, clear of the pins.
4. Switch 12 V on, then connect USB.
5. Never plug or unplug a motor while 12 V is on.

## First power-up with firmware

Each stage only needs the one before it to have worked.

1. **Logic only** (Pico on USB, drivers on `VIO`, no 12 V, no motors): `owlet ping`, `owlet caps`.
   `owlet enable` should pull `EN` low on both drivers, and `owlet move 20 0` should light the
   activity LED while it moves. `owlet disable` returns `EN` high.
2. **12 V on, no motors:** set both current limits (step 2 of the bring-up order below).
3. **Motors connected:** point the head forward, `owlet enable`, then small moves
   (`owlet move 10 0 --wait`). Pan + must turn right and tilt + must tilt up; if not, flip
   `kPanReversed` / `kTiltReversed` in `main.cpp`, or swap the two wires of one coil.
4. **Limits:** find the safe travel by hand and store it with `owlet cal`.

## Reserved for later

| Pico pin   | Purpose |
|------------|---------|
| GP4, GP5   | TMC2209 UART (current and microstep setup in software, stall detection) |
| GP10, GP11 | Pan and tilt home switches |
| GP14, GP15 | Separate MS1/MS2 for the tilt driver, if per-axis step size is ever wanted |
