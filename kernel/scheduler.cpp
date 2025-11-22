/**
 *
 */
#include "cornishrtk.hpp"
#include "port.h"
#include "port_traits.h"

#include <algorithm>
#include <array>
#include <atomic>
#include <bit>
#include <bitset>
#include <cassert>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <ctime>
#include <limits>
#include <span>

// I hate macros but oh well
#define DEBUG_PRINT_ENABLE 1
#if DEBUG_PRINT_ENABLE
# define DEBUG_PRINT(fmt, ...) \
     std::printf("[tick=%08u] " fmt "\n", port_tick_now(), ##__VA_ARGS__)
#else
# define DEBUG_PRINT(...) ((void)0)
#endif


namespace rtk
{
   static void DEBUG_DUMP_READY_QUEUE(const char* where);

   static constexpr uint32_t UINT32_BITS = std::numeric_limits<uint32_t>::digits;
   static constexpr uint8_t IDLE_THREAD_ID = 1; // Reserved
   static constexpr Thread::Priority IDLE_PRIORITY(MAX_PRIORITIES - 1);

   static constexpr std::size_t align_down(std::size_t size, std::size_t align) { return size & ~(align - 1); }
   static constexpr std::size_t align_up(std::size_t size, std::size_t align)   { return (size + (align - 1)) & ~(align - 1); }

   struct TaskControlBlock
   {
      uint32_t id;
      enum class State : uint8_t { Ready, Running, Sleeping, Blocked} state{State::Ready};
      uint8_t priority;
      Tick    wake_tick{0};

      std::span<std::byte> stack;
      Thread::Entry entry;

      // Intrusive queue/array links
      TaskControlBlock* next{nullptr};
      TaskControlBlock* prev{nullptr};
      uint16_t sleep_index{UINT16_MAX};

      // Opaque, in-place port context storage
      alignas(RTK_PORT_CONTEXT_ALIGN) std::array<std::byte, RTK_PORT_CONTEXT_SIZE> context_storage{};
      port_context_t* context() noexcept { return reinterpret_cast<port_context_t*>(context_storage.data()); }

      TaskControlBlock(uint32_t id, Thread::Priority priority, std::span<std::byte> stack, Thread::Entry entry) :
         id(id), priority(priority), stack(stack), entry(entry) {}
   };

   struct StackLayout
   {
      TaskControlBlock* tcb;
      std::span<std::byte> tls_region;
      std::span<std::byte> user_stack;

      explicit StackLayout(std::span<std::byte> const buffer, std::size_t const tls_bytes)
      {
         auto const base = reinterpret_cast<std::uintptr_t>(buffer.data());
         auto const end  = base + buffer.size();

         // TCB at very top, aligned down
         auto const tcb_start = align_down(end - sizeof(TaskControlBlock), alignof(TaskControlBlock));
         tcb = reinterpret_cast<TaskControlBlock*>(tcb_start);

         // TLS just below TCB
         auto const tls_size = align_up(tls_bytes, alignof(std::max_align_t));
         auto const tls_top  = tcb_start;
         auto const tls_base = align_down(tls_top - tls_size, alignof(std::max_align_t));

         assert(tls_base >= base && "Buffer too small for TLS+TCB");

         auto const tls_offset = static_cast<std::size_t>(tls_base - base);
         auto const tls_length = static_cast<std::size_t>(tls_top  - tls_base);
         tls_region = buffer.subspan(tls_offset, tls_length); // zero-length span if tls_bytes == 0

         // User stack: everything below TLS
         auto const stack_len = static_cast<std::size_t>(tls_base - base);
         assert(stack_len > 64 && "Buffer too small after carving TCB/TLS");

         user_stack = buffer.subspan(0, stack_len);
      }
   };

   class ReadyMatrix
   {
      class RoundRobinQueue
      {
         TaskControlBlock* head{nullptr};
         TaskControlBlock* tail{nullptr};

      public:
         [[nodiscard]] constexpr bool empty() const noexcept { return !head; }
         [[nodiscard]] constexpr TaskControlBlock* front() const noexcept { return head; }
         [[nodiscard]] constexpr bool has_peer() const noexcept { return head && head != tail; }

         void push_back(TaskControlBlock* tcb) noexcept
         {
            assert(tcb->next == nullptr && tcb->prev == nullptr && "TCB already linked");
            tcb->next = nullptr;
            tcb->prev = tail;
            if (tail) tail->next = tcb; else head = tcb;
            tail = tcb;
         }
         TaskControlBlock* pop_front() noexcept
         {
            if (!head) return nullptr;
            auto* tcb = head;
            head = tcb->next;
            if (head) head->prev = nullptr; else tail = nullptr;
            tcb->next = tcb->prev = nullptr;
            return tcb;
         }
         void remove(TaskControlBlock* tcb) noexcept
         {
            if (tcb->prev) tcb->prev->next = tcb->next; else head = tcb->next;
            if (tcb->next) tcb->next->prev = tcb->prev; else tail = tcb->prev;
            tcb->next = tcb->prev = nullptr;
         }
         // This walks the linked list so isn't 'free'. Will be used for debugging only for now
         [[nodiscard]] size_t size() const noexcept
         {
            std::size_t n = 0;
            for (auto* tcb = head; tcb; tcb = tcb->next) ++n;
            return n;
         }
      };

      std::array<RoundRobinQueue, MAX_PRIORITIES> matrix{};
      uint32_t bitmap{0};
      static_assert(MAX_PRIORITIES <= UINT32_BITS, "bitmap cannot hold that many priorities!");

   public:
      [[nodiscard]] constexpr bool empty() const noexcept { return bitmap == 0; }
      [[nodiscard]] constexpr bool empty_at(uint32_t priority) const noexcept
      {
         return matrix[priority].empty();
      }
      [[nodiscard]] std::size_t size_at(uint32_t priority) const noexcept
      {
         return matrix[priority].size();
      }
      [[nodiscard]] constexpr bool has_peer(uint32_t priority) const noexcept
      {
         return matrix[priority].has_peer();
      }
      [[nodiscard]] constexpr int best_priority() const noexcept
      {
         return bitmap ? std::countr_zero(bitmap) : -1;
      }
      [[nodiscard]] std::bitset<UINT32_BITS> bitmap_view() const noexcept
      {
         return {bitmap};
      }

      void enqueue_task(TaskControlBlock* tcb) noexcept
      {
         assert(tcb->next == nullptr && tcb->prev == nullptr && "duplicate enqueue");
         matrix[tcb->priority].push_back(tcb);
         bitmap |= (1u << tcb->priority);
      }
      TaskControlBlock* pop_highest_task() noexcept
      {
         if (bitmap == 0) return nullptr;
         auto const priority = std::countr_zero(bitmap);
         TaskControlBlock* tcb = matrix[priority].pop_front();
         if (matrix[priority].empty()) bitmap &= ~(1u << priority);
         return tcb;
      }
      [[nodiscard]] TaskControlBlock* peek_highest_task() const noexcept
      {
         if (bitmap == 0) return nullptr;
         auto const priority = std::countr_zero(bitmap);
         return matrix[priority].front();
      }
      void remove_task(TaskControlBlock* tcb) noexcept
      {
         auto const priority = tcb->priority;
         matrix[priority].remove(tcb);
         if (matrix[priority].empty()) bitmap &= ~(1u << priority);
      }
   };

   class SleepMinHeap
   {
      std::array<TaskControlBlock*, MAX_THREADS> heap_buffer{};
      uint16_t size_count{0};

      static uint16_t parent(uint16_t i) noexcept { return (i - 1u) >> 1; }
      static uint16_t left  (uint16_t i) noexcept { return (i << 1) + 1u; }
      static uint16_t right (uint16_t i) noexcept { return (i << 1) + 2u; }

      static bool earlier(const TaskControlBlock* a, const TaskControlBlock* b) noexcept
      {
         return a->wake_tick.is_before(b->wake_tick);
      }
      void swap_nodes(uint16_t a, uint16_t b) noexcept
      {
         std::swap(heap_buffer[a], heap_buffer[b]);
         heap_buffer[a]->sleep_index = a;
         heap_buffer[b]->sleep_index = b;
      }
      void sift_up(uint16_t i) noexcept
      {
         while (i > 0) {
            uint16_t pnt = parent(i);
            if (!earlier(heap_buffer[i], heap_buffer[pnt])) break;
            swap_nodes(i, pnt);
            i = pnt;
         }
      }
      void sift_down(uint16_t i) noexcept
      {
         while (true) {
            uint16_t lft = left(i), rht = right(i), mid = i;
            if (lft < size_count && earlier(heap_buffer[lft], heap_buffer[mid])) mid = lft;
            if (rht < size_count && earlier(heap_buffer[rht], heap_buffer[mid])) mid = rht;
            if (mid == i) break;
            swap_nodes(i, mid);
            i = mid;
         }
      }

   public:
      [[nodiscard]] bool empty() const noexcept { return size_count == 0; }
      [[nodiscard]] uint16_t size() const noexcept { return size_count; }
      [[nodiscard]] TaskControlBlock* top() const noexcept
      {
         return size_count ? heap_buffer[0] : nullptr;
      }

      void push(TaskControlBlock* tcb) noexcept
      {
         uint16_t i = size_count++;
         heap_buffer[i] = tcb;
         tcb->sleep_index = i;
         sift_up(i);
      }
      TaskControlBlock* pop_min() noexcept
      {
         if (!size_count) return nullptr;
         TaskControlBlock* tcb = heap_buffer[0];
         tcb->sleep_index = UINT16_MAX;
         --size_count;
         if (size_count) {
            heap_buffer[0] = heap_buffer[size_count];
            heap_buffer[0]->sleep_index = 0;
            sift_down(0);
         }
         return tcb;
      }
      void remove(TaskControlBlock* tcb) noexcept
      {
         uint16_t i = tcb->sleep_index;
         if (i == UINT16_MAX) return; // not in heap
         tcb->sleep_index = UINT16_MAX;
         --size_count;
         if (i == size_count) return; // removed last

         heap_buffer[i] = heap_buffer[size_count];
         heap_buffer[i]->sleep_index = i;

         // Re-heapify from i (either direction)
         if (i > 0 && earlier(heap_buffer[i], heap_buffer[parent(i)])) {
            sift_up(i);
         } else {
            sift_down(i);
         }
      }
   };

   class AtomicDeadline
   {
      static constexpr uint32_t DISARMED = std::numeric_limits<uint32_t>::max();
      std::atomic_uint32_t deadline{DISARMED};

   public:
      [[nodiscard]] bool armed(std::memory_order mo = std::memory_order_relaxed) const noexcept
      {
         return deadline.load(mo) != DISARMED;
      }
      [[nodiscard]] bool due(Tick now, std::memory_order mo = std::memory_order_relaxed) const noexcept
      {
         uint32_t dline = deadline.load(mo);
         return dline != DISARMED && now.has_reached(dline);
      }
      void store(Tick tick, std::memory_order mo = std::memory_order_relaxed) noexcept
      {
         deadline.store(tick.value(), mo);
      }
      void disarm() noexcept
      {
         deadline.store(DISARMED, std::memory_order_relaxed);
      }
   };

   struct InternalSchedulerState
   {
      std::atomic_uint32_t next_thread_id{2}; // 0 is invalid, 1 is reserved for idle, 2..N are user threads
      std::atomic_bool     preempt_disabled{true};
      std::atomic_bool     need_reschedule{false};
      TaskControlBlock* current_task{nullptr};
      TaskControlBlock* idle_tcb{nullptr};

      ReadyMatrix ready_matrix;
      SleepMinHeap sleepers;
      AtomicDeadline next_wake_tick; // When next sleeper must be woken
      AtomicDeadline next_slice_tick; // When the next RR rotation is due
   };
   static constinit InternalSchedulerState iss;

   static void set_task_ready(TaskControlBlock* tcb)
   {
      DEBUG_PRINT("enqueue id=%u prio=%u", tcb->id, tcb->priority);
      tcb->state = TaskControlBlock::State::Ready;
      iss.ready_matrix.enqueue_task(tcb);
      // If the new task is higher priority than the running one, we need to reschedule.
      if (!iss.current_task || tcb->priority < iss.current_task->priority) {
         iss.need_reschedule.store(true, std::memory_order_relaxed);
      }
   }

   static void context_switch_to(TaskControlBlock* next)
   {
      if (next == iss.current_task) return;
      TaskControlBlock* previous_task = iss.current_task;
      iss.current_task = next;
      if (previous_task) previous_task->state = TaskControlBlock::State::Ready; // Task might have already been set ready but ensure it is
      iss.current_task->state = TaskControlBlock::State::Running;

      DEBUG_PRINT("context_switch_to(): previous=%u -> next=%u", previous_task ? previous_task->id : 0u, next->id);

      port_set_thread_pointer(next);
      port_switch(previous_task ? previous_task->context() : nullptr, iss.current_task->context());

      DEBUG_PRINT("context_switch_to(): returned to scheduler");
   }

   static void schedule()
   {
      if (iss.preempt_disabled.load(std::memory_order_acquire)) return;
      if (!iss.need_reschedule.exchange(false)) return;
      DEBUG_DUMP_READY_QUEUE("schedule() snapshot");

      auto const now = Scheduler::tick_now();

      // Wake sleepers
      while (true) {
         auto* top_tcb = iss.sleepers.top();
         if (!top_tcb || now.is_before(top_tcb->wake_tick)) break; // Noone asleep or not due yet
         (void)iss.sleepers.pop_min();
         DEBUG_PRINT("wake       id=%u -> wake_tick=%u", top_tcb->id, top_tcb->wake_tick.value());
         set_task_ready(top_tcb);
      }

      // Update when the next sleeper will wake up
      if (auto* top_tcb = iss.sleepers.top()) {
         iss.next_wake_tick.store(top_tcb->wake_tick);
      } else {
         iss.next_wake_tick.disarm();
      }

      // Round robin rotation
      if (iss.next_slice_tick.due(now) &&
          iss.current_task &&
          iss.current_task != iss.idle_tcb &&
          iss.current_task->state == TaskControlBlock::State::Running &&
          iss.ready_matrix.has_peer(iss.current_task->priority)) // Don't requeue for singular threads at a priority level
      {
         DEBUG_PRINT("timeslice  rotate id=%u", iss.current_task->id);
         set_task_ready(iss.current_task);
      }

      // Choose next runnable
      auto* next_task = iss.ready_matrix.pop_highest_task();
      if (!next_task) {
         iss.next_slice_tick.disarm();
         if (iss.current_task != iss.idle_tcb) {
            DEBUG_PRINT("pick idle");
            context_switch_to(iss.idle_tcb);
         }
         return;
      }

      // If we are preempted by a higher thread, push current back to queue
      if (iss.current_task &&
          iss.current_task != iss.idle_tcb && // Idle does not belong in the ready matrix
          iss.current_task != next_task &&
          iss.current_task->state == TaskControlBlock::State::Running)
      {
         set_task_ready(iss.current_task);
      }

      if (next_task->priority == IDLE_PRIORITY) {
         iss.next_slice_tick.disarm();
      } else {
         // Serve a fresh slice
         iss.next_slice_tick.store(now + TIME_SLICE);
      }
      DEBUG_PRINT("pick       id=%u prio=%u", next_task->id, next_task->priority);
      context_switch_to(next_task);
   }

   static void thread_trampoline(void* arg_void)
   {
      auto* tcb = static_cast<TaskControlBlock*>(arg_void);
      tcb->entry();
      // If user function returns, park/surrender forever (could signal joiners later)
      while (true) Scheduler::yield();
   }

   alignas(RTK_STACK_ALIGN) static std::array<std::byte, 2048> idle_stack{}; // TODO: size stack
   static void idle_entry(void*) { while(true) port_idle(); }

   void Scheduler::init(uint32_t tick_hz)
   {
      port_init(tick_hz);
      // Create the idle thread
      StackLayout slayout(idle_stack, 0);
      iss.idle_tcb = ::new (slayout.tcb) TaskControlBlock(
         IDLE_THREAD_ID,
         IDLE_PRIORITY,
         slayout.user_stack,
         Thread::Entry(idle_entry)
      );

      port_context_init(iss.idle_tcb->context(),
                        slayout.user_stack.data(),
                        slayout.user_stack.size(),
                        thread_trampoline,
                        iss.idle_tcb);
      // Note: we intentionally do NOT call set_task_ready(idle_tcb).
      // Idle is never in ready_matrix.
   }

   void Scheduler::start()
   {
      DEBUG_DUMP_READY_QUEUE("start() entry");
      auto* first = iss.ready_matrix.pop_highest_task();
      assert(first != nullptr);
      DEBUG_DUMP_READY_QUEUE("start() head picked/removed");
      iss.current_task = first;
      iss.current_task->state = TaskControlBlock::State::Running;
      iss.next_slice_tick.store(Scheduler::tick_now() + TIME_SLICE);
      iss.preempt_disabled.store(false, std::memory_order_release); // Open the scheduler

      port_set_thread_pointer(iss.current_task); // I think this is correct?
      port_start_first(iss.current_task->context());

      if constexpr (RTK_SIMULATION) {
         while (true) {
            schedule();
            struct timespec req{.tv_nsec = 1'000'000};
            nanosleep(&req, nullptr);
         }
      }
      __builtin_unreachable();
   }

   void Scheduler::yield()
   {
      DEBUG_PRINT("yield by   id=%u", iss.current_task->id);
      if (iss.current_task && iss.current_task->state == TaskControlBlock::State::Running) {
         set_task_ready(iss.current_task); // put current at tail
      }
      rtk_request_reschedule();
      port_yield();
   }

   Tick Scheduler::tick_now()
   {
      return Tick(port_tick_now());
   }

   void Scheduler::sleep_for(uint32_t ticks)
   {
      if (ticks == 0) { yield(); return; }
      // Put current to sleep
      assert(iss.current_task && "no current thread");
      assert(iss.current_task->sleep_index == UINT16_MAX && "thread is already sleeping");
      iss.current_task->state = TaskControlBlock::State::Sleeping;
      iss.current_task->wake_tick = tick_now() + (ticks);
      DEBUG_PRINT("sleep      id=%u until=%u (+%u)", iss.current_task->id, iss.current_task->wake_tick.value(), ticks);
      iss.sleepers.push(iss.current_task);

      if (auto const* top_tcb = iss.sleepers.top()) {
         iss.next_wake_tick.store(top_tcb->wake_tick);
      }

      rtk_request_reschedule();
      port_yield();
   }

   void Scheduler::preempt_disable() { port_preempt_disable(); iss.preempt_disabled.store(true, std::memory_order_release); }
   void Scheduler::preempt_enable()  { iss.preempt_disabled.store(false, std::memory_order_release); port_preempt_enable(); }

   Thread::Thread(Entry entry, std::span<std::byte> stack, Priority priority)
   {
      assert(priority < IDLE_PRIORITY);

      auto id = iss.next_thread_id.fetch_add(1, std::memory_order_relaxed);

      StackLayout slayout(stack, 0);
      tcb = ::new (slayout.tcb) TaskControlBlock(id, priority, slayout.user_stack, entry);

      port_context_init(tcb->context(), slayout.user_stack.data(), slayout.user_stack.size(), thread_trampoline, tcb);
      set_task_ready(tcb);

      DEBUG_DUMP_READY_QUEUE("Thread() created");
   }

   Thread::~Thread()
   {
      if (!tcb) return;
      port_context_destroy(tcb->context());
      tcb->~TaskControlBlock();
      tcb = nullptr;
   }

   [[nodiscard]] Thread::Id Thread::get_id() const noexcept
   {
      return tcb->id;
   }

   std::size_t Thread::reserved_stack_size()
   {
      auto tcb_size = align_up(sizeof(TaskControlBlock), alignof(TaskControlBlock));
      auto tls_size = 0; // TODO
      return tcb_size + tls_size;
   }

   struct MutexImpl
   {
      // Implicitly initialized to nullptr when opaque is constinit'ed
      TaskControlBlock* owner;
      TaskControlBlock* wait_head;
      TaskControlBlock* wait_tail;

      constexpr MutexImpl() = default;

      [[nodiscard]] bool has_waiters() const noexcept { return wait_head != nullptr; };

      // Assumes preemption is already disabled
      bool try_lock_under_guard()
      {
         auto* current = iss.current_task;
         assert(current && "Mutex::try_lock() with no current task");

         if (owner != nullptr) return false;
         owner = current;
         return true;
      }

      void enqueue_waiter(TaskControlBlock* tcb) noexcept
      {
         // TCB must not be in any other intrusive list
         assert(tcb->next == nullptr && tcb->prev == nullptr);

         tcb->next = nullptr;
         tcb->prev = wait_tail;

         if (wait_tail) {
            wait_tail->next = tcb;
         } else {
            wait_head = tcb;
         }
         wait_tail = tcb;
      }

      TaskControlBlock* pop_waiter() noexcept
      {
         TaskControlBlock* tcb = wait_head;
         if (!tcb) return nullptr;

         wait_head = tcb->next;
         if (wait_head) {
            wait_head->prev = nullptr;
         } else {
            wait_tail = nullptr;
         }
         tcb->next = nullptr;
         tcb->prev = nullptr;
         return tcb;
      }
   };
   static_assert(std::is_trivially_constructible_v<MutexImpl>, "Reinterpreting opaque block is now UB");
   static_assert(std::is_standard_layout_v<MutexImpl>, "Reinterpreting opaque block is now UB");
   static_assert(sizeof(MutexImpl) == sizeof(Mutex::ImplStorage), "Adjust forward size declaration in header to match");
   static_assert(alignof(MutexImpl) == alignof(Mutex::ImplStorage), "Adjust forward align declaration in header to match");

   [[nodiscard]] bool Mutex::is_locked() const noexcept { return self->owner != nullptr; }

   void Mutex::lock()
   {
      {
         Scheduler::Lock guard;
         if (self->try_lock_under_guard()) return;

         auto* current = iss.current_task;
         assert(self->owner != current && "Mutex::lock() called recursively by owner");

         DEBUG_PRINT("mutex lock: id=%u blocked on mutex %p", current->id, (void*)this);

         current->state = TaskControlBlock::State::Blocked;
         self->enqueue_waiter(current);

         rtk_request_reschedule();
      }

      // Actually block
      port_yield();
   }

   bool Mutex::try_lock()
   {
      Scheduler::Lock guard;
      return self->try_lock_under_guard();

   }

   bool Mutex::try_lock_for(Tick::Delta timeout)
   {
      return try_lock_until(Scheduler::tick_now() + timeout);
   }

   bool Mutex::try_lock_until(Tick deadline)
   {
      while (true) {
         Tick::Delta remaining;
         {
            Scheduler::Lock guard;
            if (self->try_lock_under_guard()) return true;
            auto const now = Scheduler::tick_now();
            if (now.has_reached(deadline)) return false;
            remaining = deadline - now;
         }
         Scheduler::sleep_for(remaining);
      }
   }

   void Mutex::unlock()
   {
      Scheduler::Lock guard;

      auto* current = iss.current_task;
      assert(current && "Mutex::unlock() with no current task");
      assert(self->owner == current && "Mutex::unlock() by non-owner");

      if (!self->has_waiters()) {
         self->owner = nullptr;
         return;
      }

      TaskControlBlock* next_owner = self->pop_waiter();
      assert(next_owner);
      self->owner = next_owner;

      DEBUG_PRINT("mutex unlock: id=%u -> waking waiter id=%u on mutex %p",
                  current->id, next_owner->id, (void*)this);

      next_owner->state = TaskControlBlock::State::Ready;
      set_task_ready(next_owner);
   }


   extern "C" void rtk_on_tick(void)
   {
      DEBUG_PRINT("rtk_on_tick()");
      auto const now = Scheduler::tick_now();
      if (iss.next_wake_tick.due(now) || iss.next_slice_tick.due(now)) {
         DEBUG_PRINT("rtk_on_tick() -> %s%s", iss.next_wake_tick.due(now) ? "wake_tick due " : "", iss.next_slice_tick.due(now) ? "slice_tick due" : "");
         rtk_request_reschedule();
      }
   }

   extern "C" void rtk_request_reschedule(void)
   {
      DEBUG_PRINT("rtk_request_reschedule()");
      iss.need_reschedule.store(true, std::memory_order_relaxed);
      // Under simulation we should return to the scheduler loop in user context
      // On bare metal, we should pend a software interrupt and return from this ISR
   }


   static void DEBUG_DUMP_READY_QUEUE(const char*)
   {
      std::printf("[tick=%08u] bitmap: %s", port_tick_now(), iss.ready_matrix.bitmap_view().to_string().c_str());
      for (unsigned p = 0; p < MAX_PRIORITIES; ++p) {
         if (!iss.ready_matrix.empty_at(p)) {
            std::printf(" p%u(%zu)", p, iss.ready_matrix.size_at(p));
         }
      }
      std::printf(" current_task_id=%u\n", iss.current_task ? iss.current_task->id : 0);
   }

} // namespace rtk
