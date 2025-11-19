/**
 * Cornish Real-Time Kernel Application Programming Interface
 *
 *
 *
 *
*/
#ifndef _CORNISH_RTK_HPP_
#define _CORNISH_RTK_HPP_

#include <cstddef>
#include <cstdint>
#include <limits>
#include <span>
#include <type_traits>

namespace rtk
{
   //-------------- Config ---------------
   static constexpr uint32_t MAX_PRIORITIES = 32; // 0 = highest, 31 = lowest
   static constexpr uint32_t MAX_THREADS    = 64;
   static constexpr uint32_t TIME_SLICE     = 10; // In ticks

   static_assert(MAX_PRIORITIES <= std::numeric_limits<uint32_t>::digits, "Unsupported configuration");

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
      using Id = std::uint32_t;

      struct Entry
      {
         using Fn = void(*)(void*);
         Fn fn;
         void* arg;
         explicit Entry(Fn fn, void* arg = nullptr) : fn(fn), arg(arg) {}
         void operator()() const { fn(arg); }
      };

      struct Priority
      {
         std::uint8_t val;
         constexpr explicit Priority(std::uint8_t v) : val(v) {}
      };

      Thread(Entry entry, std::span<std::byte> stack, Priority priority);
      ~Thread();

      [[nodiscard]] Id get_id() const noexcept;

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
      // Future deadline wraps naturally
      friend constexpr Tick operator+(Tick tick, Delta delta) noexcept
      {
         return Tick(tick.t + delta);
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
         return lhs.t - rhs.t;
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
