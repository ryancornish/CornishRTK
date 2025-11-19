#include <cornishrtk.hpp>

#include <array>
#include <cstdint>
#include <iostream>

alignas(16) static std::array<std::byte, 16 * 1024> fast_stack{};
alignas(16) static std::array<std::byte, 16 * 1024> slow_stack{};
alignas(16) static std::array<std::byte, 16 * 1024> logger_stack{};

static void fast_worker(void* arg)
{
   const char* name = static_cast<const char*>(arg);
   std::uint32_t counter = 0;

   while (true)
   {
      std::cout << "[FAST ] " << name << " count=" << counter++ << "\n";
      // Runs fairly often. high priority so it preempts others.
      rtk::Scheduler::sleep_for(10);
   }
}

static void slow_worker(void* arg)
{
   const char* name = static_cast<const char*>(arg);
   std::uint32_t counter = 0;

   while (true)
   {
      std::cout << "[SLOW ] " << name << " count=" << counter++ << "\n";
      rtk::Scheduler::sleep_for(25);
   }
}

static void logger_worker(void* /*arg*/)
{
   std::uint32_t tick_block = 0;

   while (true)
   {
      auto now = rtk::Scheduler::tick_now().value();
      std::cout << "[LOG  ] heartbeat at tick=" << now
                << " (block=" << tick_block++ << ")\n";

      // Sleep long enough that higher-priority work dominates,
      // but occasionally yield cooperatively to show both paths.
      rtk::Scheduler::sleep_for(50);

      // Explicit cooperative yield inside our own timeslice
      // (on real MCU this would just pend a switch).
      rtk::Scheduler::yield();
   }
}

int main()
{
   rtk::Scheduler::init(10);

   // Priorities: lower number = higher priority.
   //   fast:   prio 1 (preempts others)
   //   slow:   prio 2
   //   logger: prio 10 (only runs when others are sleeping)
   rtk::Thread fast_thread(rtk::Thread::Entry(fast_worker, (void*)"fast_worker"), fast_stack, rtk::Thread::Priority(1));

   rtk::Thread slow_thread(rtk::Thread::Entry(slow_worker, (void*)"slow_worker"), slow_stack, rtk::Thread::Priority(2));

   rtk::Thread logger_thread(rtk::Thread::Entry(logger_worker), logger_stack, rtk::Thread::Priority(10));

   rtk::Scheduler::start();
   return 0;
}
