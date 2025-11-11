#include <array>
#include <atomic>
#include <cstdlib>
#include <iostream>
#include <port.h>

static std::atomic<bool> need_reschedule{false};
static port_context_t* current_context;
static port_context_t* task1_context;
static port_context_t* task2_context;


extern "C" void rtk_on_tick(void)            { need_reschedule.store(true, std::memory_order_relaxed); }
extern "C" void rtk_request_reschedule(void) { need_reschedule.store(true, std::memory_order_relaxed); }

static port_context_t* alloc_port_context()
{
   size_t const size      = 56; // Hard coded right now
   size_t const alignment = 8;  // ^
   size_t const allocate  = ((size + alignment - 1) / alignment) * alignment;

   return static_cast<port_context_t*>(std::aligned_alloc(alignment, allocate));
}

static void free_port_context(port_context_t* port_context_handle)
{
   free(port_context_handle);
}

static void pick_and_switch()
{
   static bool flip = false;
   port_context_t* next = flip ? task1_context : task2_context;
   flip = !flip;
   if (next != current_context) {
      port_switch(&current_context, next);
      current_context = next;
   }
}

static void thread_fn(void* arg)
{
   std::string name(static_cast<const char*>(arg));
   while (true)
   {
      std::cout << name << std::endl;
      // Pretend to work a bit
      for (long long i = 0; i < 300'000'000; ++i) { (void)0; }

      port_yield(); // Ask scheduler to switch
   }
}

// stacks (Boost continuation backend currently ignores these, that’s fine)
alignas(16) static std::array<uint8_t, 16*1024> stack1;
alignas(16) static std::array<uint8_t, 16*1024> stack2;

int main()
{
   port_init(1000);

   // allocate opaque contexts
   task1_context = alloc_port_context();
   task2_context = alloc_port_context();
   if (!task1_context || !task2_context) { std::cout << "System error: Failed to allocate port contexts" << std::endl; return 1; }

   port_context_init(task1_context, stack1.data(), stack1.size(), thread_fn, reinterpret_cast<void*>(const_cast<char*>("T1 tick")));
   port_context_init(task2_context, stack2.data(), stack2.size(), thread_fn, reinterpret_cast<void*>(const_cast<char*>("T2 tick")));

   current_context = task1_context;

   // start first thread; after it returns (on yield), run a tiny scheduler loop
   port_start_first(task1_context); // enters T1; on yield we’re back here

   while (true)
   {
      if (need_reschedule.exchange(false)) pick_and_switch();
      // a tiny pause avoids pegging the host CPU in this sim
      for (int i = 0; i < 10000; ++i) { (void)0; }
   }

   free_port_context(task1_context);
   free_port_context(task2_context);
   return 0;
}
