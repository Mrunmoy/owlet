# owlet Python client

```bash
pip install ./tools/python          # from the repo root; installs the `owlet` command
owlet ping                           # finds the head's serial port by itself
owlet enable
owlet move 30 -15 --wait
owlet pos
owlet watch --hz 20                  # Ctrl-C to stop
owlet dtc                            # what has gone wrong since power-up, and what to check
owlet stat                           # health counters
owlet selftest                       # exercise both motors and report
owlet shell                          # interactive: `move 10 10`, `speed 45`, ...
```

```python
from owlet import Owlet

with Owlet() as head:                # or Owlet("/dev/ttyACM1"), Owlet("COM5")
    head.enable()
    head.speed(60)
    head.move(45, 10, wait=True)
    print(head.position())
    head.disable()
```

The head also reports things by itself. Connect a function and it is called when they happen; your
program does not have to ask:

```python
with Owlet() as head:
    head.arrived.connect(lambda pan, tilt: print("arrived at", pan, tilt))
    head.fault_changed.connect(lambda axis, faults: print(axis, "driver faults:", faults))
    head.limit_hit.connect(lambda axis, end: print(axis, "hit its", end, "switch"))
    head.enable()
    head.move(30, -15)               # returns at once
    ...
```

Connecting the first function starts a background reader. A program that connects none gets no extra
thread. `Owlet(dispatch="manual")` never starts one; call `head.process_events()` from your own loop.

| Also | |
|---|---|
| `head.trouble_codes()` / `head.clear_trouble_codes()` | what went wrong, as a list of `Trouble` |
| `head.stats()` | events raised and lost, late control-loop passes |
| `head.seek_home(wait=True)` | find the reference from the limit switches, on a head that has them |
| `head.diag("pan")` | what a motor driver reports: current, load, faults |
| `Tracker`, `PixelController` | camera tracking; see the main README |

The only dependency is `pyserial`. The full list of signals and commands is in the repository's main
README; the protocol is specified in `docs/DESIGN.md` §5.
