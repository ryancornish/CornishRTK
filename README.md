# Cyros

Cyros is a small real-time kernel for embedded systems, written in modern C++.

The idea it is built around: an RTOS should be something you can *read* before
you *trust*. The core is deliberately small, the layer boundaries are strict,
and everything beyond scheduling and blocking is a component you opt into.

> **Status: work in progress.** The architecture described here exists and is
> under test, but APIs still move. See [Status](#status) for what is real
> today.

## Design ethos

**The kernel has no concept of time.** Most RTOS cores read "real-time" as
"the scheduler counts ticks" and end up with timekeeping tangled through
everything. In Cyros the kernel knows whether a thread is ready, running, or
blocked, and what it is blocked on. That is all. Time lives in a separate
driver that sits beside the kernel, not beneath it, and anything time-flavoured
is built by composing the two. If your system needs no clock, you can build a
valid Cyros with no time driver at all.

**Derived truth over cached state.** Where the kernel needs a value that other
cores can change, it computes the value at the point of use instead of storing
a copy and propagating updates. The clearest example is priority inheritance:
there is no stored "effective priority" anywhere. A thread's urgency is
computed from its base priority and the waiters actually queued on the locks it
actually holds, at the moment the scheduler asks. Cross-core notifications
carry no values either, only the fact that a core should look again. A stale
notification costs a redundant scheduling pass, never a wrong answer.

**Pay only for what you enable.** Features are separate components with their
own headers and sources, selected at build time. There is no master
configuration header trying to anticipate every use case, and a feature you do
not select costs you nothing in code size or in reading.

**The hosted build is the real kernel.** The scheduler, the blocking machinery,
and the synchronisation primitives run identically on a desktop and on a
target, because the port boundary is cut below all of them. The test suites run
the actual kernel, under genuine SMP and genuine asynchronous preemption, on a
development machine.

## Architecture

Four layers. Each depends only on what sits below it, and the kernel and the
time driver deliberately do not depend on each other.

```
        +-------------------------------------------------+
        |  userlib                                        |
        |  opt-in features: sync primitives, round-robin  |
        +-------------------------------------------------+
                 |                          |
                 v                          v
        +-------------------+      +-------------------+
        |  kernel           |      |  time driver      |
        |  per-core         |      |  monotonic time,  |
        |  schedulers,      |      |  scheduled        |
        |  threads,         |      |  callbacks        |
        |  waitables,       |      |  (periodic,       |
        |  mutex core       |      |  tickless, sim)   |
        +-------------------+      +-------------------+
                 |                          |
                 v                          v
        +-------------------------------------------------+
        |  port  (C ABI)                                  |
        |  contexts, interrupt & preemption control,      |
        |  core startup, inter-core signalling, time src  |
        +-------------------------------------------------+
```

### Port

The port is the hardware (or host) abstraction, exposed as a plain C ABI so it
can be implemented in C or assembly. It provides opaque CPU contexts and the
switch between them, interrupt masking and preemption control as balanced
save/restore pairs, multi-core startup, an inter-core reschedule signal, and
the low-level time source.

The cut is placed so that everything above it is policy and everything below
it is mechanism. The kernel never sees a fiber, a pthread, or a signal frame,
only a context it asks the port to create, switch, and destroy.

Two Linux ports exist today:

- **linux_coop** builds contexts on raw fcontext switching. Cooperative,
  simple, and fast to debug against.
- **linux_preempt** delivers genuine asynchronous preemption in a hosted
  process. Cores are threads, interrupts are POSIX signals, and a running
  thread can be preempted between any two instructions, exactly the asynchrony
  a hardware interrupt provides. It is built on
  [sigctx](external/sigctx), a small self-contained C library for capturing
  and resuming CPU contexts from signal handlers, developed alongside Cyros
  and vendored here.

Ports declare their scheduling model, and the kernel adapts. The intended
first bare-metal target is ARM Cortex-M.

### Kernel

The kernel is a set of per-core schedulers in an SMP arrangement, plus the
blocking machinery that connects them.

Scheduling is fixed-priority and per-core. Every thread is pinned to one core,
chosen at registration from its affinity mask, and priorities are ordered
globally across the system rather than per core. Core-local scheduler state is
only ever touched by its owning core. When another core needs something done
to a thread, it records the request as a flag on the thread itself and rings
the owning core with an inter-core signal. There are no cross-core message
queues and no payloads in flight, so there is nothing to fill up and nothing
to deliver out of order.

Blocking is built on one abstraction, the **waitable**. A waitable owns a
priority-ordered queue of parked threads and a single derived-class hook that
answers whether the calling thread can proceed without blocking. Wakes come in
barge-permitting flavours (`wake_one`, `wake_all`) and a committed-handoff
flavour (`wake_one_and_transfer`) for primitives that must not let a late
arrival barge past a woken waiter. A thread can block on several waitables at
once with `wait_on_any`.

Mutual exclusion is not built on the waitable. The kernel carries a separate
**mutex core** (`base_mutex`) with ownership, barge-free handoff, and priority
inheritance built in. Mutexes are deliberately excluded from group waits, and
the type system enforces that rather than a runtime check. The split keeps the
general waitable cheap: primitives that never own anything never pay for
inheritance machinery.

Priority inversion is handled by two protocols, chosen per lock at the
declaration site:

- **Inheritance** (`sync::mutex`). A holder's urgency is raised to its most
  urgent waiter, transitively through chains of waiting holders, with the
  chain walk bounded at a fixed depth.
- **Immediate ceiling** (`sync::cemutex`). The holder runs at the lock's
  ceiling priority from acquisition to release, whether or not anyone is
  waiting. Cheaper to evaluate and more predictable, at the price of paying
  the boost on every acquisition.

Both are folds over current truth, computed when the scheduler picks, never
stored, so there is no propagation and nothing to go stale.

Thread lifetime also runs through the scheduler. A thread ends by returning
from its entry function or by calling `this_thread::thread_exit()`, either way
the scheduler retires it on its own core, releases its resources, and signals
joiners through an ordinary waitable. Stacks are caller-owned buffers, and the
kernel states its minimum stack size as a compile-time constant.

### Time driver

The time driver depends on the port and nothing else. It provides a monotonic
`time_point`, durations, and one-shot or recurring scheduled callbacks. Three
implementations ship: a periodic (tickful) driver, a tickless driver, and a
simulation driver that makes host-side tests deterministic.

Because the kernel does not know the driver exists, replacing it with your own
timer infrastructure means implementing one interface, not excavating the
scheduler.

### userlib

User-facing features, each one optional:

- **sync**: `mutex` and `cemutex` as thin facades over the kernel's mutex
  core, and a counting `semaphore` built on the waitable.
- **round_robin**: time-slicing for equal-priority threads, built by composing
  the time driver with the scheduler's public surface. The kernel needed no
  changes to support it, which is the layering doing its job: even the classic
  "scheduler feature" is an optional component outside the scheduler.

## A taste of the API

```cpp
#include <cyros/kernel/kernel.hpp>
#include <cyros/kernel/thread.hpp>
#include <cyros/sync/mutex.hpp>

using namespace cyros;

sync::mutex log_lock;

std::array<std::byte, 32 * 1024> stack;

int main()
{
   kernel::initialise();

   thread worker(
      []{
         log_lock.lock();
         // ...
         log_lock.unlock();
      },
      stack,
      thread::priority(4)
   );

   kernel::start();
}
```

Threads take a caller-owned stack, a priority (lower number is more urgent),
and optionally a core-affinity mask. `this_thread::` provides `id()`,
`priority()`, `yield()`, `thread_exit()`, and the blocking entry points
`wait_on` / `wait_on_any`. Critical sections and preemption control are RAII
guards under `this_core::`. Building a new blocking primitive means deriving
from `waitable` and implementing one hook.

## Methodology

The unit suites run against both Linux ports, so every kernel test executes
once cooperatively and once under real asynchronous preemption. A separate
port-contract suite pins down the port ABI's guarantees (mask ownership,
nesting and overlap of interrupt and preemption regions) independently of the
kernel that relies on them. Consumer tests build small complete applications
against a packaged Cyros to keep the public headers honest.

The working rules are simple to state. Concurrency claims are tested under the
preemptive port, not argued from the source. Fixes are verified against the
failure they fix, ideally by re-introducing the defect and watching the test
catch it. Costs that matter are measured, and bounds that matter are written
down next to the code that has them.

## Building

Cyros is assembled by [Cyros-Builder](../cyros-builder), a companion tool that
reads the component and profile descriptions in this repository and produces a
packaged static library plus headers for a chosen port, time driver, and
feature set. The `component.toml` and `feature.toml` files throughout the tree
and the profiles under `build/profiles/` exist for it. Build instructions and
the profile format live in that project.

## Status

Implemented and under test:

- Per-core SMP scheduling with fixed priorities, core affinity, and global
  priority ordering.
- Threads: creation onto caller-owned stacks, termination through the
  scheduler, join built on an ordinary waitable.
- The waitable model, group waits, and the committed-handoff wake.
- The kernel mutex core with priority inheritance (transitive, bounded) and
  immediate-ceiling protocols, surfaced as `sync::mutex` and `sync::cemutex`.
- A lock-free blocking-chain query with advisory deadlock detection, reading
  the same published state the scheduler folds urgency from.
- A counting semaphore.
- Round-robin time-slicing as an opt-in feature.
- The `chrono` feature: an `alarm` waitable, `sleep_for`/`sleep_until`, and
  timed semaphore acquisition, all composed over the time driver.
- Both Linux ports, including preemptive scheduling driven by POSIX signals.
- Time drivers: periodic, tickless, simulation.

Planned:

- A bare-metal port, ARM Cortex-M first.
- Migration as an explicit user API (threads are pinned today).
- More userlib features, including an optional allocator.
- Example projects and on-device demos.

If you are reading the scheduler or considering writing a port, feedback is
welcome.
