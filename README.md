# WORK IN PROGRESS!

# CornishRTK
CornishRTK (Cornish Real-Time Kernel) is a compact, single-core, preemptive scheduler and synchronization kernel written in modern C++.
It provides just enough kernel primitives to build or host higher-level threading systems - including the ability to run std::thread, std::mutex, and related C++ standard library features via the GCC gthreads API.

## Architectural Overview
 ┌──────────────────────────────────────────────────────────┐
 │                 Application/User Layer                   │
 │                                                          │
 │   ┌────────────────────────────────────────────────────┐ │
 │   │           C++ Self-Contained Kernel API            │ │
 │   │  • kapi::Thread                                    │ │
 │   │  • kapi::Mutex                                     │ │
 │   │  • kapi::Semaphore                                 │ │
 │   │  • kapi::Scheduler                                 │ │
 │   │  For direct interface to the kernel                │ │
 │   └────────────────────────────────────────────────────┘ │
 │                                                          │
 │   ┌────────────────────────────────────────────────────┐ │
 │   │                C gthreads API                      │ │
 │   │  • __gthread_create / __gthread_join               │ │
 │   │  • __gthread_mutex_* / __gthread_once              │ │
 │   │  • __gthread_key_*  (TSS)                          │ │
 │   │  Enables libstdc++ std::thread, std::mutex, etc.   │ │
 │   └────────────────────────────────────────────────────┘ │
 └──────────────────────────────────────────────────────────┘
              │                    │
              │                    │
              ▼                    ▼
 ┌──────────────────────────────────────────────────────────┐
 │                    CornishRTK Kernel                     │
 │   • Preemptive, priority-based scheduler                 │
 │   • Ready / wait queues                                  │
 │   • Time-slicing and tick handling                       │
 │   • Mutex, semaphore, scheduler lock                     │
 │   • C++ implementation, no dynamic allocation            │
 └──────────────────────────────────────────────────────────┘
              │
              ▼
 ┌──────────────────────────────────────────────────────────┐
 │                    Porting Layer (C)                     │
 │   • Context init / switch (port_context_t)               │
 │   • Tick source and interrupt hooks                      │
 │   • Preemption enable / disable                          │
 │   • ISR enter / exit                                     │
 │   Backends (TODO):                                       |
 |       • Linux (Boost.Context / libucontext) TODO,        │
 │       • ARM Cortex-M (PendSV/SysTick) TODO,              │
 │       • LEON / SPARC,                                    |
 |       • RISC-V                                           │
 └──────────────────────────────────────────────────────────┘
              ▼
        ┌────────────────────────┐
        │        Hardware        │
        │   CPU + Timer + IRQ    │
        └────────────────────────┘


## Ports

### Linux

#### Boost.Context

Prerequisite
- Ubuntu/Debian: `sudo apt install libboost-context-dev`
- Arch: `sudo pacman -S boost`
- macOS (Homebrew): `brew install boost`