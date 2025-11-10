/**
 * Cornish Real-Time Kernel Application Programming Interface
 *
 *
 *
 *
*/
#ifndef _CORNISH_RTK_HPP_
#define _CORNISH_RTK_HPP_

#include <cstdint>
#include <cstddef>
#include <type_traits>

#ifndef RTK_SIZEOF_PORT_CONTEXT_T
# error "RTK_SIZEOF_PORT_CONTEXT_T must be defined on the command line (e.g. -DRTK_SIZEOF_PORT_CONTEXT_T=64)."\
        "Calculate this by peeking the 'sizeof(port_context_t)' evaluation within the port implementation."
#endif
#ifndef RTK_ALIGNOF_PORT_CONTEXT_T
# error "RTK_ALIGNOF_PORT_CONTEXT_T must be defined on the command line (e.g. -DRTK_ALIGNOF_PORT_CONTEXT_T=8)."\
        "Calculate this by peeking the 'alignof(port_context_t)' evaluation within the port implementation."
#endif

namespace rtk
{
   //-------------- Config ---------------
   static constexpr uint32_t MAX_PRIORITIES = 32; // 0 = highest, 31 = lowest
   static constexpr uint32_t MAX_THREADS    = 64;
   static constexpr uint32_t TIME_SLICE     = 10; // In ticks

   static_assert(MAX_PRIORITIES <= UINT32_WIDTH, "Unsupported configuration");

   struct Scheduler
   {
      static void init(uint32_t tick_hz);
      static void start();
      static void yield();
      static class Tick tick_now();
      static void sleep_for(uint32_t ticks);  // cooperative sleep (sim)

      static void preempt_disable();
      static void preempt_enable();

      struct Lock
      {
         Lock()  { preempt_disable(); }
         ~Lock() { preempt_enable(); }
      };
   };

   class Thread
   {
      struct TaskControlBlock* tcb;

   public:
      using EntryFunction = void(*)(void*);
      Thread(EntryFunction fn, void* arg, void* stack_base, std::size_t stack_size, uint8_t priority);
      ~Thread();

      static std::size_t reserved_stack_size();
   };


   class Mutex;

   class Semaphore;

   class ConditionVar;


   // TODO: The footprint of this class has grown quite big. Is it cleaner to keep in in a separate header? My assumption is 'probably not'
   class Tick
   {
      uint32_t t{0};

      static constexpr bool after_eq_u32(uint32_t a, uint32_t b) noexcept
      {
         constexpr uint32_t HALF = 1u << 31;
         return uint32_t(a - b) < HALF; // a >= b (wrap-safe)
      }

   public:
      using Delta = uint32_t;

      constexpr Tick() noexcept = default;
      constexpr explicit Tick(uint32_t value) noexcept : t(value) {}

      [[nodiscard]] constexpr uint32_t value() const noexcept { return t; }

      // Wrap-safe "now >= deadline"
      [[nodiscard]] constexpr bool has_reached(uint32_t deadline) const noexcept
      {
         return after_eq_u32(t, deadline);
      }
      [[nodiscard]] constexpr bool has_reached(Tick deadline) const noexcept
      {
         return after_eq_u32(t, deadline.t);
      }
      // Wrap-safe "now < deadline"
      [[nodiscard]] constexpr bool is_before(uint32_t deadline) const noexcept
      {
         return !after_eq_u32(t, deadline);
      }
      [[nodiscard]] constexpr bool is_before(Tick deadline) const noexcept
      {
         return !after_eq_u32(t, deadline.t);
      }

      // ---- Arithmetic ----
      // Future deadline: wraps naturally
      friend constexpr Tick operator+(Tick tick, Delta delta) noexcept
      {
         return Tick(uint32_t(tick.t + delta));
      }
      friend constexpr Tick operator+(Delta delta, Tick tick) noexcept
      {
         return tick + delta;
      }
      Tick& operator+=(Delta delta) noexcept
      {
         t += delta; return *this;
      }

      // Elapsed ticks between two instants (mod 2^32)
      friend constexpr Delta operator-(Tick lhs, Tick rhs) noexcept
      {
         return uint32_t(lhs.t - rhs.t);
      }

      // ---- Comparators vs uint32_t/Tick deadlines (wrap-safe '>=' and '<') ----
      friend constexpr bool operator>=(Tick now, uint32_t deadline) noexcept
      {
         return now.has_reached(deadline);
      }
      friend constexpr bool operator<(Tick now, uint32_t deadline) noexcept
      {
         return now.is_before(deadline);
      }
      friend constexpr bool operator>=(Tick now, Tick deadline) noexcept
      {
         return now.has_reached(deadline.t);
      }
      friend constexpr bool operator<(Tick now, Tick deadline) noexcept
      {
         return now.is_before(deadline.t);
      }
   };
   static_assert(sizeof(Tick) == sizeof(uint32_t), "It is important that Tick be as cheap as uint32_t");
   static_assert(std::is_trivially_copyable_v<Tick>, "test");



} // namespace rtk

#endif
