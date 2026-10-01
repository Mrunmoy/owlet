# owlet Python client

```bash
pip install ./tools/python          # from the repo root; installs the `owlet` command
owlet ping                           # auto-detects the ESP32-S3 port
owlet enable
owlet move 30 -15 --wait
owlet pos
owlet watch --hz 20                  # Ctrl-C to stop
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

The only dependency is `pyserial`. The protocol is specified in `docs/DESIGN.md` §5.
