# Event-driven owlet: design

*Status: written 2026-10-05 as a proposal, revised the same day with the owner's decisions (section 12), and implemented in firmware 0.5.0 except where section 11 says otherwise.*

## 1. Why

Today the firmware asks. Every 2 ms the protocol looks at the head to see whether it has arrived, whether the watchdog fired, whether it is time for a position report. Driver status is fetched on a timer and sits in a cache until someone requests `$DIAG`. The PC has to poll for anything that is not one of three hard-wired messages.

That worked for two servos. It does not scale to what is coming: limit switches, homing, stall detection, driver faults, perhaps a second transport. Each would add another thing for every layer to ask about, and another place to forget to.

The aim is the opposite arrangement. Whatever notices a change announces it once. Whoever cares has connected to it. Nothing above the hardware edge asks "has anything happened?", on the device or on the PC.

## 2. Principles

1. **One context owns all state.** Motion, protocol and homing run in the main loop and nowhere else. No locks, no shared state with interrupts, and host tests stay exact.
2. **Events carry changes, not data streams.** "The head arrived", "a limit was hit", "the pan driver reports over-temperature". Serial bytes and the 2 ms motion step are not events; they stay direct calls. Routing them through a queue would cost time and gain nothing.
3. **Safety does not wait in a queue.** What stops a motor is a direct, synchronous path. The event tells everyone else afterwards.
4. **Sampling stops at the driver.** A value with no interrupt (a register in the TMC2209) is read by the driver that owns it and turned into an event when it changes. Nothing above that driver reads it on a timer.
5. **No heap, no `std::function`, no virtual event objects.** Fixed sizes, decided at build time.

## 3. The pieces

```
 interrupts            main loop (the only owner of state)                        PC
 ──────────            ─────────────────────────────────────                     ────
 limit switch ──post─┐
 stall pin    ──post─┤   ┌────────────┐    ┌────────────┐   ┌─ PanTilt (stop, invalidate)
 2 ms timer ──flag─┐ ├──▶│ EventQueue │───▶│ dispatcher │──▶├─ Homing
                   │ │   └────────────┘    └────────────┘   ├─ TextProtocol ── $EVT,… ──▶ client ──▶ your handlers
 PanTilt, homing,  │ │        ▲                             └─ ActivityIndicator
 driver monitor ───┼─┘        │ post (same queue)
                   ▼
              motion tick  (direct call, not an event)
```

### 3.1 `Event`

One plain struct, 8 bytes, trivially copyable.

```cpp
enum class EventId : uint8_t
{
    Arrived, WatchdogExpired,                         // motion
    LimitHit, Homed, HomingFailed, PositionLost,      // reference
    Stalled, DriverFaultChanged, DriverOnline, DriverOffline,   // drivers
    BusFault, BusOk, EventsLost,                      // system
};

struct Event
{
    EventId  id;
    uint8_t  axis;     // 0 = pan, 1 = tilt, 0xFF = the whole head
    uint16_t arg;      // meaning depends on id: fault bits, which end, failure code
    uint32_t timeMs;   // when it was posted; the bus stamps it
};
```

There is no event for "started moving": the activity light already follows the head's state and nothing else needs it.

A tagged struct was chosen over typed signals (`Signal<Args...>` with slot arrays). Typed signals read nicely but each instantiation costs code on a Cortex-M0+, they cannot be put in one queue, and a test cannot simply compare two of them. A flat struct can be queued, logged, compared and sent down the wire with one small table.

### 3.2 `EventQueue<N>` (built as the ring inside `core::EventBus`)

A ring of `N` events (16 to start: 128 bytes). `post()` may be called from an interrupt or from the loop. It takes a few instructions inside an interrupt lock; the RP2040's core has no atomic compare-and-swap, so a brief mask is both the simplest correct choice and the usual one. The lock is an injected `hal::IInterruptLock`, a no-op on the host.

Overflow is a rule, not an accident. A queue alone cannot promise that a limit hit is never lost: when it is full, something has to give. So the conditions that must not be lost do not depend on the queue at all.

- **Latched conditions.** A limit hit, a stall and "events were lost" each set a bit outside the queue, from the interrupt or the loop. The dispatcher turns a set bit into an event as soon as there is room, and clears it. A full queue delays the report; it cannot lose it.
- **Everything else is queued**, and if the queue is full it is dropped and counted. The next slot to come free carries an `EventsLost` with the count, so the PC knows to re-read state.
- **Repeats collapse.** A `DriverFaultChanged` for an axis that already has one waiting overwrites it instead of taking a second slot. Limit, stall, watchdog and homing events are never merged.

Position reports are not events. The protocol produces them from the tick, as now.

### 3.3 Dispatch

```cpp
class IEventObserver
{
public:
    virtual ~IEventObserver() = default;
    virtual void onEvent(const Event& event) = 0;
};

template <std::size_t N> class EventBus;   // owns the queue, holds std::array<IEventObserver*, N>
```

In practice nobody writes `onEvent` or posts a raw event. Two thin classes give every event a name at both ends:

```cpp
class PanTilt : ..., private EventNotifier        // announces
{
    ...
    notifyArrived();
    notifyPositionLost(axis);
};

class TextProtocol : ..., public EventDelegate    // reacts
{
    void onArrived() override;
    void onLimitHit(uint8_t axis, LimitEnd end) override;
};
```

`EventNotifier` turns `notifyXxx(...)` into a posted event; `EventDelegate` turns a delivered event back into `onXxx(...)`, and a class overrides only the ones it wants. The same rule names the protocol's commands, one `handleXxx` each. So: **`handleXxx`** does what was asked, **`notifyXxx`** says what happened, **`onXxx`** reacts.

Observers are attached once, in `main.cpp` during start-up, in the order they should hear events: motion policy first, then homing, then protocols, then indicators. The order is part of the design: a protocol must see the state *after* the motion policy has reacted. Observers may post new events from a handler; those are delivered after the current one, never inside it, so there is no re-entrancy.

### 3.4 The loop

```
wait for the 2 ms timer flag (sleeping, not spinning)
tick motion                      ← direct call; may post Arrived, WatchdogExpired
let drivers do one bounded step  ← may post DriverFaultChanged, Stalled
deliver every queued event
feed received bytes to the protocol
```

`ControlLoop` keeps this shape, so its existing tests keep their meaning.

## 4. Stopping on a limit

A limit switch tripping outside homing means the soft limits did not hold, which means the step count is already wrong. Two things must happen: stepping stops, and the position is marked not trusted (`PositionLost`), after which only homing or an explicit `ENABLE,<pan>,<tilt>` restores it.

**Decided (owner): the switch's interrupt stops the pulses directly.** The reaction must not depend on when the loop next runs.

- `IStepGenerator` gains `halt()`, safe to call from an interrupt, and `resume()`. On the Pico, `halt()` disables that axis's PIO state machine and empties its FIFO: the pulse train stops within microseconds, mid-burst.
- The interrupt does two things and returns: `halt()`, and set the latched limit bit for that axis and end. No arithmetic, no queue walk.
- The loop then sees the latch, delivers `LimitHit`, and the motion policy posts `PositionLost` for the axis. A burst was cut short, so the count is unknown; that is accepted, because the position was already untrustworthy.

Two more parts make this complete:

- **A level check in the motion path.** An edge interrupt cannot fire for a switch that is already pressed at power-up, or one that was pressed while interrupts were off for a flash write. So before each burst the axis also asks a guard whether it may step that way, and the guard reads the switch level. This costs one pin read per tick and closes both gaps.
- **Homing uses the same halt, on purpose.** While homing seeks a switch, that switch is expected. The interrupt still halts the pulses on the edge, which gives a reference accurate to a single step instead of to one tick, and homing calls `resume()` and carries on. The halt is only a fault when no homing asked for it.

Two positions were weighed. For the interrupt: a stepper driven into a stop loses sync at once, so only an immediate cut protects the mechanism. For the level check alone: it is simpler and identical on every board, and the interrupt could come later. The design above takes the interrupt as the primary path and keeps the level check for the cases an edge cannot cover.

## 5. Step timing

Step pulses within a burst are already exact: PIO produces them. The risk is the gap *between* bursts if the loop is late. The loop keeps one burst queued behind the one running, so it has a full tick of slack. That depth becomes a setting of the generator (1 or 2 bursts ahead) instead of a constant: more slack costs the same amount of command latency. No interrupt is needed for this; the PIO's own FIFO is the queue.

Three rules keep the loop on time, and each becomes a counter in `$STAT` so a violation is visible:

- No handler blocks for more than a fraction of a tick. The TMC2209 exchange (about 1 ms today) is split so that sending and receiving are separate steps.
- Calibration is not saved to flash while an axis is moving. The erase stops everything for tens of milliseconds, so `CAL,SAVE` is refused with `NAK,CAL,MOV`.
- A late tick is counted, with the worst lateness kept.

Moving the planner into a timer interrupt was considered and rejected. It needs soft-float in an interrupt, makes every command a critical section, and does not survive a flash write either, because interrupts are off for that.

## 6. On the wire

One new unsolicited line:

```
$EVT,<name>,<axis>,<arg>*CK        $EVT,LIMIT,PAN,MAX     $EVT,STALL,TILT,0     $EVT,FAULT,PAN,6
                                   $EVT,HOMED,-,0         $EVT,LOST,PAN,0       $EVT,OVERFLOW,-,3
```

- **It is additive.** The protocol stays v3. Clients already ignore lines they do not expect, so an existing client keeps working. The camera-tracking project is not disturbed.
- **`$ARRIVED`, `$TIMEOUT` and `$ERR,BUS` stay exactly as they are.** They become three observers' output instead of three special cases in a polling function.
- **`$EVENTS,<0|1>`** turns the `$EVT` lines off or on (default on), for a client that wants a quiet link.
- **`$STAT`** reports loop and queue health: events posted, dropped, the queue's high-water mark, late ticks, worst lateness.
- Driver values (`$DIAG`) can still be read on request; `$EVT,FAULT` means nobody needs to.

## 7. On the PC

```python
head = Owlet()
head.stalled.connect(lambda axis: print(axis, "stalled"))
head.limit_hit.connect(on_limit)
head.fault_changed.connect(on_fault)
head.arrived.connect(on_arrived)
head.move(30, -15)            # returns at once; arrival comes as a signal
```

A *handler* is a function in your own program that the client calls when the head reports something: `on_limit` above runs when a limit switch trips. It is how a program reacts to the head without asking it.

- **Nothing changes for a program that connects no handlers.** `Owlet` stays synchronous, as it is today, for scripts and the command line.
- Connecting the first handler starts one background reader thread for that connection. It reads only between the program's own requests: a request holds the port from its write to its reply, so the reader can never take that reply.
- Handlers run on that reader thread, or on the caller's thread when the line arrives during one of its own calls. `Owlet(dispatch="manual")` starts no thread: handlers then run during the program's calls and `head.process_events()`, for GUI loops and for programs that want no threads of their own.
- `move(..., wait=True)` and `wait_arrived()` remain, built on the same signal. The existing `on_event` hook stays as the low-level way in.

## 8. Testing

- The event bus runs on the host with a no-op lock. A test posts an event, calls `dispatch()`, and checks what each observer did. Nothing is timing-dependent.
- A recording observer captures the exact sequence, so a scenario reads as its event list: `LimitHit(pan, max), PositionLost(pan)`.
- `halt()` is tested on a fake generator: called mid-burst it stops the count, and the latch produces exactly one `LimitHit`.
- Overflow, ordering, and "an observer posts during dispatch" get their own tests.
- The Python side is tested against `owlet_sim`, which gains a way to inject a limit hit and a fault.
- All existing tests (300 when this was written; 368 C++ now) keep passing at every step of section 11.

## 9. Building more than one

- **One board file per head.** Pins, polarities, steps per revolution, belt ratio, speeds and switch positions move out of `main.cpp` into a single constexpr description, so a second board is one new file.
- **`owlet selftest` grows with the hardware:** switch wiring (does each one read pressed and released), homing repeatability, and the counters from `$STAT`.
- **The event log is the diagnostic.** With `$EVT` on, a session transcript shows everything the head noticed, in order, with device timestamps.

## 10. Stall detection, when it comes

It is one more event source and needs no new mechanism. The TMC2209's load value rises with speed, so the driver monitor compares it against a threshold that scales with the commanded rate, and posts `Stalled(axis)` when the load falls below it for several readings in a row while moving. The motion policy treats it as it treats a limit: stop, position not trusted. The driver's DIAG pin can raise the same event from an interrupt if the wire is added.

## 11. Order of work

Each step leaves every test passing and the head working.

1. **Done.** `Event`, `EventBus`, `IInterruptLock`, with tests.
2. **Done.** `PanTilt` posts `Arrived` and `WatchdogExpired`; bus faults become `BusFault` / `BusOk`. `TextProtocol` is an observer and `onTick` only schedules the position stream. The bytes on the wire did not change. `tick()` still returns its status to a direct caller; protocols no longer receive it.
3. **Done.** `Tmc2209Monitor` tells a listener when its chip comes, goes or changes its faults; `core::AxisEventRelay` turns that into `DriverOnline`, `DriverOffline`, `DriverFaultChanged` (drivers do not depend on the event types). `$EVT`, `$EVENTS`, `$STAT` and the Python signals are in. `owlet_sim --inject` raises a limit hit, a fault, an offline driver or a stall from its standard input. Still to do from this step: a check with the motor supply on that the open-coil bits do not make `FAULT` chatter between standstill and motion.
4. **Done in code; waiting for the switches.** `IStepGenerator::halt()` / `resume()`; `core::LimitTrip` runs in a switch's interrupt, halts the pulses and posts `LimitHit`; `StepperAxis` withholds steps toward a pressed switch, and on finding its generator halted ends the move, marks itself blocked and resumes the generator; `PanTilt` turns a blocked axis into `PositionLost` and refuses motion until `SEEK` or a fresh `ENABLE,<pan>,<tilt>`; `HeadHoming` homes tilt then pan. In the Pico app all of it sits behind `kLimitSwitchesFitted = false`. Two switches per axis (owner, 2026-10-05). The one change from the plan: the axis, not the homing routine, resumes a halted generator, so homing needs no special case.
5. **Done, bar one item.** `CAL,SAVE` is refused while moving and `$STAT` carries the loop counters. Splitting the TMC2209 exchange is not done. First measurement on the Pico, with the motor supply off so the drivers were only being probed: no late pass in five seconds idle and two moves (longest gap 3 ms), one late pass of 4 ms during a second of velocity mode, and a 13 ms gap for a flash save at rest. The 4 ms pass is probably a driver probe waiting out its reply timeout, which is the blocking exchange this item is about; it needs measuring again with the drivers powered before deciding how to split it.
6. **Board file done:** `apps/pico-owlet/board.hpp` holds the pins, polarities, mechanism, driver settings, speeds, limits and switch pins. Self-test additions (switch wiring, homing repeatability, the `$STAT` counters) wait for the switches, since two of the three need them.
7. Stall detection.

Headers touched along the way get their documentation converted to Doxygen, in the header only.

## 12. Decisions

| # | Question | Decision |
|---|---|---|
| 1 | Limit reaction: a check in the motion path, or an interrupt that halts the pulses? | **Interrupt** (owner, 2026-10-05), so the reaction does not wait for the loop. The level check is kept alongside it for a switch already pressed. Section 4. |
| 2 | Homing at power-up: automatic, or on command? | **On command for now; automatic once it has earned trust** (owner, 2026-10-05). It will be a stored setting, off by default, because automatic homing means the head moves by itself at switch-on. |
| 3 | Python: when does the reader thread exist, and where do handlers run? | **Built as proposed (client 0.4.0):** no thread until a handler is connected, then handlers on the reader thread, with a manual mode (`dispatch="manual"`). Section 7. |

## 13. What will not be built

- A general publish/subscribe framework with runtime registration.
- Events for bytes, for the motion step, or for position reports.
- A second thread or an RTOS task for motion, or the motion planner in an interrupt.
- Dynamic memory anywhere in the firmware.

## 14. Alternatives that were weighed

- **The motion planner in a timer interrupt.** Dropped: flash writes stall interrupts on the RP2040, and the planner's state would then be shared between an interrupt and the loop. What it was meant to buy, slack against a late loop, came instead from keeping one burst queued in the PIO's own FIFO (section 5).
- **Must-not-lose conditions as queue entries.** Dropped in favour of latched bits outside the queue (section 3.2), so a full queue can never lose a limit hit.
- **One event per fault report.** Repeated fault events now merge.
- **A reader thread in the Python client from the start.** The client stays synchronous until a handler is connected (section 7).
- **A "motion started" event.** Dropped as noise; arrival is the event that matters.
- **The limit reaction:** see section 4.
