# Pan-tilt wiring: Pico + 2 x TMC2209 + 2 x NEMA 17

Diagram: `wiring.png` (or `wiring.svg`). Regenerate with `python3 make_wiring.py`, then `convert -density 96 wiring.svg wiring.png`.

The firmware for this wiring is `apps/pico-owlet` (`./build.py --pico --flash`). The pin numbers
below are the constants in `apps/pico-owlet/board.hpp`; change both together.

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
| Pico GP4 (pin 6) and GP5 (pin 7) | PDN/UART on the **pan** driver, all three joined | TMC2209 UART for pan: TX and RX on one line. |
| Pico GP0 (pin 1) and GP1 (pin 2) | PDN/UART on the **tilt** driver, all three joined | TMC2209 UART for tilt. |

The firmware uses the two UART lines to set each motor's current (so the potentiometers no
longer matter), to select the microstep size, and to read load and fault status: `owlet diag`.
Each driver has a private line, so the drivers need no distinct addresses and MS1/MS2 stay
shared. The drivers only answer while the motor supply is on; without it the head still runs,
from the MS pins and the potentiometers. No series resistor is needed: the Pico only drives
the line while it is sending.

The LED on the Pico board itself (GP25) is the heartbeat: one short flash a second while the
firmware is running. It needs no wiring.

The driver pins labelled UART (the fifth) and CLK stay unconnected. The Pico is powered from USB only.

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
   `kPanReversed` / `kTiltReversed` in `board.hpp`, or swap the two wires of one coil.
4. **Limits:** find the safe travel by hand and store it with `owlet cal`.

## Limit switches (not fitted yet)

Two per axis, one at each end of travel. The firmware supports them but leaves these pins
alone until `kLimitSwitchesFitted` is set to `true` in `apps/pico-owlet/board.hpp`.

| Pico pin | Switch |
|----------|--------|
| GP10 | Pan, negative end (the camera's left) |
| GP11 | Pan, positive end (the camera's right) |
| GP14 | Tilt, negative end (down) |
| GP15 | Tilt, positive end (up) |

**Wire each switch normally-closed, between its pin and ground.** Use the switch's `COM` and
`NC` terminals; leave `NO` unconnected. No resistor is needed: the Pico's own pull-up is
switched on. The pin reads low while the switch is at rest and goes high when it is pressed.
It also goes high if a wire breaks or a plug comes out, so a wiring fault stops the axis
instead of letting it run into the end of its travel.

A switch wired normally-open works too: change `kSwitchPolarity` in `board.hpp` to
`ActiveLow`. A broken wire then goes unnoticed, which is why normally-closed is preferred.

Mount each switch so its lever is pressed a few degrees **before** the mechanism's hard stop,
and so the moving part can travel a little past the trip point without crushing the lever.

Fitting order, once they arrive:

1. Wire one switch. With `kLimitSwitchesFitted` still `false`, nothing changes; check the
   wiring with a multimeter (closed at rest, open when pressed).
2. Wire all four, set `kLimitSwitchesFitted = true`, and flash.
3. With the motors off, press each switch by hand and run `owlet dtc`: a switch that is read
   shows up as `30 LIMIT` on the right axis and end.
4. `owlet seek`. The head finds both ends of tilt, then pan, and stops at the middle.

