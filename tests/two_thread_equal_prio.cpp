#include <cornishrtk.hpp>
#include <iostream>
#include <cstdint>
#include <array>

// Keep stacks here so the Thread ctor can hand them to the port
alignas(16) static std::array<std::byte, 16 * 1024> t1_stack{};
alignas(16) static std::array<std::byte, 16 * 1024> t2_stack{};

static void worker(void* arg)
{
   while (true)
   {
      std::cout << static_cast<char const*>(arg) << "\n";
      rtk::Scheduler::sleep_for(20); // Tune for testing
   }
}

int main()
{
   rtk::Scheduler::init(10); // Tune for testing

   // Equal priority -> round-robin
   rtk::Thread t1(rtk::Thread::Entry(worker, (void*)"T1 tick"), t1_stack, rtk::Thread::Priority(2));
   rtk::Thread t2(rtk::Thread::Entry(worker, (void*)"T2 tock"), t2_stack, rtk::Thread::Priority(2));

   // In simulation, start() returns into an internal loop and never exits main.
   // On MCU ports, start() will not return (noreturn path).
   rtk::Scheduler::start();
   return 0;
}
