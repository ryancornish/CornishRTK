/**
 * port_linux_boost_context.cpp
 */
#include <port.h>
#include <port_traits.h>

#include <boost/context/fiber.hpp>
#include <boost/context/preallocated.hpp>
#include <boost/context/stack_context.hpp>

#include <atomic>
#include <csignal>
#include <cstdint>
#include <cstring>
#include <sys/time.h>

static constinit std::atomic<uint32_t> global_tick{0};
uint32_t port_tick_now() { return global_tick.load(std::memory_order_relaxed); }

// Unused/unimplemented port stubs
void port_isr_enter() {}
void port_isr_exit(int) {}
void port_preempt_disable() {}
void port_preempt_enable() {}
void port_irq_disable() {}
void port_irq_enable() {}

struct port_context
{
  boost::context::fiber thread; // thread fiber (owned by scheduler when idle)
  boost::context::fiber sched;  // scheduler fiber (owned by thread when running)
  void*        stack_top;
  std::size_t  stack_size;
  port_entry_t entry;
  void*        arg;
  bool         started;
};

static_assert(RTK_PORT_CONTEXT_SIZE  == sizeof(port_context_t), "Adjust port_traits.h definition to match");
static_assert(RTK_PORT_CONTEXT_ALIGN == alignof(port_context_t), "Adjust port_traits.h definition to match");
static_assert(RTK_STACK_ALIGN == 16);

// thread-local "am I inside a thread?"
static thread_local port_context* tls_current = nullptr;

// No-op stack allocator for preallocated memory
struct preallocated_stack_noop
{
  using traits_type = boost::context::stack_traits;
  boost::context::stack_context allocate(std::size_t) { std::abort(); }
  void deallocate(boost::context::stack_context&) noexcept {}
};

void port_context_init(port_context_t* context,
                       void* stack_base,
                       std::size_t stack_size,
                       port_entry_t entry,
                       void* arg)
{
   // Construct/emplace port_context_t object within user-provided stack
   ::new (context) port_context{
      .thread    = {},
      .sched     = {},
      .stack_top = static_cast<std::uint8_t*>(stack_base) + stack_size,
      .stack_size= stack_size,
      .entry     = entry,
      .arg       = arg,
      .started   = false,
   };

   // Build a fiber bound to the user-provided stack.
   boost::context::stack_context boost_stack_context = {
      .size = context->stack_size,
      .sp   = context->stack_top,
   };
   boost::context::preallocated boost_prealloc(boost_stack_context.sp, boost_stack_context.size, boost_stack_context);
   preallocated_stack_noop stack_allocator;

   context->thread = boost::context::fiber(std::allocator_arg, boost_prealloc, stack_allocator,
      [context](boost::context::fiber&& sched_in) mutable -> boost::context::fiber
      {
         // First entry, save the scheduler fiber handle
         context->sched = std::move(sched_in);

         while (true) {
            tls_current = context;
            context->entry(context->arg); // Enter user code
            tls_current = nullptr;

            // Park back on scheduler until resumed again
            context->sched = std::move(context->sched).resume();
            // When resumed, we loop and re-enter user code
         }
      });

   context->started = true; // Constructed and ready (not actually running yet!)
}

void port_context_destroy(port_context_t* context)
{
   // If the thread fiber still exists, try to unwind cooperatively
   if (context->thread) {
      // Ask the fiber to finish by giving it a chance to run a tiny trampoline
      context->thread = std::move(context->thread).resume_with(
         [](boost::context::fiber&& fb){ return boost::context::fiber{}; } // Return an empty fiber -> done
      );
   }
   context->~port_context();
}

static thread_local void* global_thread_pointer = nullptr;
void  port_set_thread_pointer(void* tp) { global_thread_pointer = tp; }
void* port_get_thread_pointer(void)     { return global_thread_pointer; }

// Switch into 'to' (thread). Returns when the thread yields
void port_switch(port_context_t* /*from*/, port_context_t* to)
{
   tls_current = to;
   // Enter/resume the thread fiber. Returns when thread yields back
   to->thread = std::move(to->thread).resume();
   tls_current = nullptr;
}

// Start the very first thread
void port_start_first(port_context_t* first)
{
   tls_current = first;
   first->thread = std::move(first->thread).resume(); // Run until first yield
   tls_current = nullptr;
}

// Thread calls this to yield to scheduler
void port_yield()
{
   if (!tls_current) {
      // Special case when there is no current just reschedule!
      rtk_request_reschedule();
      return;
   }

   port_context* current = nullptr;
   std::swap(current, tls_current);
   // Yield to the stored scheduler fiber; the returned fiber
   // is the thread's handle updated for the next resume.
   current->sched = std::move(current->sched).resume();
}

void port_idle()
{
   // Sleep 1 ms (to simulate power saving) then yield cooperatively
   struct timespec req{.tv_nsec = 1'000'000};
   nanosleep(&req, nullptr);
   port_yield();
}


void port_init(uint32_t tick_hz)
{
   static std::array<std::uint8_t, 8 * 1024> altstack;
   stack_t ss = { .ss_sp = altstack.data(), .ss_flags = 0, .ss_size = altstack.size() };
   sigaltstack(&ss, nullptr);

   struct sigaction sa{};
   sa.sa_handler = [](int){
      global_tick.fetch_add(1, std::memory_order_relaxed);
      rtk_on_tick();
   };
   sigemptyset(&sa.sa_mask);
   sa.sa_flags = SA_ONSTACK | SA_RESTART;
   sigaction(SIGALRM, &sa, nullptr);

   itimerval it{};
   int usec = 1'000'000 / static_cast<int>(tick_hz);
   it.it_interval.tv_sec  = usec / 1'000'000;
   it.it_interval.tv_usec = usec % 1'000'000;
   it.it_value            = it.it_interval;
   setitimer(ITIMER_REAL, &it, nullptr);
}
