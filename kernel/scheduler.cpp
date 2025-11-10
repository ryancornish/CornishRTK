/**
 *
 */
#include <cornishrtk.hpp>
#include <port.h>

#include <algorithm>
#include <array>
#include <atomic>
#include <bit>
#include <bitset>
#include <cassert>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <time.h>
#include <limits>

namespace rtk
{
   static void DEBUG_DUMP_READY_QUEUE(const char* where);

   static constexpr uint32_t UINT32_BITS = std::numeric_limits<uint32_t>::digits;
   static constexpr uint32_t IDLE_PRIORITY = MAX_PRIORITIES - 1;

   static constexpr std::size_t align_down(std::size_t sz, std::size_t al) { return sz & ~(al - 1); }
   static constexpr std::size_t align_up(std::size_t sz, std::size_t al)   { return (sz + (al - 1)) & ~(al - 1); }

   struct TaskControlBlock
   {
      enum class State : uint8_t { Ready, Running, Sleeping, Blocked} state{State::Ready};
      uint8_t priority{0};
      Tick    wake_tick{0};

      void*  stack_base{nullptr};
      size_t stack_size{0};
      Thread::EntryFunction entry_fn;
      void*  arg{nullptr};

      // Intrusive queue/array links
      TaskControlBlock* next{nullptr};
      TaskControlBlock* prev{nullptr};
      uint16_t sleep_index{UINT16_MAX};

      // Opaque, in-place port context storage
      alignas(RTK_ALIGNOF_PORT_CONTEXT_T) std::array<std::byte, RTK_SIZEOF_PORT_CONTEXT_T> context_storage{};
      port_context_t* context() noexcept { return reinterpret_cast<port_context_t*>(context_storage.data()); }
   };
   static_assert(std::is_trivially_copyable_v<TaskControlBlock>, "TaskControlBlock must be trivially copyable for reset-in-place to work!");;

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
         return std::bitset<UINT32_BITS>(bitmap);
      }

      void enqueue_task(TaskControlBlock* tcb) noexcept
      {
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
      TaskControlBlock* peek_highest_task() const noexcept
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
            uint16_t p = parent(i);
            if (!earlier(heap_buffer[i], heap_buffer[p])) break;
            swap_nodes(i, p);
            i = p;
         }
      }
      void sift_down(uint16_t i) noexcept
      {
         uint16_t l, r, m;
         while (true) {
            uint16_t l = left(i), r = right(i), m = i;
            if (l < size_count && earlier(heap_buffer[l], heap_buffer[m])) m = l;
            if (r < size_count && earlier(heap_buffer[r], heap_buffer[m])) m = r;
            if (m == i) break;
            swap_nodes(i, m);
            i = m;
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
         tcb->sleep_index = 0xFFFF;
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
         if (i == 0xFFFF) return; // not in heap
         tcb->sleep_index = 0xFFFF;
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
      std::atomic_uint32_t deadline{UINT32_MAX};

   public:
      void store(Tick tick, std::memory_order mo = std::memory_order_relaxed) noexcept
      {
         deadline.store(tick.value(), mo);
      }
      [[nodiscard]] Tick load(std::memory_order mo = std::memory_order_relaxed) noexcept
      {
         return Tick(deadline.load(mo));
      }
      void disarm() noexcept { deadline.store(UINT32_MAX, std::memory_order_relaxed); }
   };

   struct InternalSchedulerState
   {
      std::atomic<bool> preempt_disabled{false};
      std::atomic<bool> need_reschedule{false};
      TaskControlBlock* current_task{nullptr};
      Thread* idle_thread{nullptr};

      ReadyMatrix ready_matrix;
      SleepMinHeap sleepers;
      AtomicDeadline next_wake_tick; // When next sleeper must be woken
      AtomicDeadline next_slice_tick; // When the next RR rotation is due



      void reset() // Should I just do a self assignment from default ctor instead?
      {
         preempt_disabled.store(false);
         need_reschedule.store(false);
         current_task = nullptr;
         idle_thread = nullptr;
         next_wake_tick.disarm();
         next_slice_tick.disarm();
      }
   };
   static constinit InternalSchedulerState iss;

   static void set_task_ready(TaskControlBlock* tcb)
   {
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
      if (previous_task) previous_task->state = TaskControlBlock::State::Ready;
      iss.current_task->state = TaskControlBlock::State::Running;

      port_set_thread_pointer(next);

      port_context_t* previous_context = previous_task ? previous_task->context() : nullptr;
      port_switch(&previous_context, iss.current_task->context());
   }

   static void schedule()
   {
      if (iss.preempt_disabled.load(std::memory_order_acquire)) return;
      if (!iss.need_reschedule.exchange(false)) return;
      DEBUG_DUMP_READY_QUEUE("schedule() need_reschedule -> true");

      auto const now = Scheduler::tick_now();

      // Wake sleepers
      while (true) {
         auto* top_tcb = iss.sleepers.top();
         if (!top_tcb || now.is_before(top_tcb->wake_tick)) break; // Not due yet
         (void)iss.sleepers.pop_min();
         set_task_ready(top_tcb);
      }

      // Update when the next sleeper will wake up
      if (auto* top_tcb = iss.sleepers.top()) {
         iss.next_wake_tick.store(top_tcb->wake_tick);
      } else {
         iss.next_wake_tick.disarm();
      }

      // Round robin rotation
      if (now.has_reached(iss.next_slice_tick.load()) &&
          iss.current_task &&
          iss.current_task->state == TaskControlBlock::State::Running &&
          iss.ready_matrix.has_peer(iss.current_task->priority)) // Don't requeue for singular threads at a priority level
      {
         set_task_ready(iss.current_task);
      }

      // Choose next runnable
      auto* next_task = iss.ready_matrix.pop_highest_task();
      if (!next_task) {
         iss.next_slice_tick.disarm();
         return; // Shhhh everyone is sleeping!
      }

      // If we are preempted by a higher thread, push current back to queue
      if (iss.current_task && iss.current_task != next_task &&
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
      context_switch_to(next_task);
   }

   alignas(16) static std::array<uint8_t, 1024> idle_stack{};
   static void idle_entry(void*) { while(true) port_idle(); }

   void Scheduler::init(uint32_t tick_hz)
   {
      iss.reset();
      port_init(tick_hz);
      // Create the idle thread
      iss.idle_thread = new Thread(idle_entry, nullptr, idle_stack.data(), idle_stack.size(), IDLE_PRIORITY);
   }

   void Scheduler::start()
   {
      DEBUG_DUMP_READY_QUEUE("start() entry");
      auto first = iss.ready_matrix.pop_highest_task();
      assert(first != nullptr);
      DEBUG_DUMP_READY_QUEUE("start() head picked/removed");
      iss.current_task = first;
      iss.current_task->state = TaskControlBlock::State::Running;
      iss.next_slice_tick.store(Scheduler::tick_now() + TIME_SLICE);

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
      iss.current_task->state = TaskControlBlock::State::Sleeping;
      iss.current_task->wake_tick = tick_now() + (ticks);
      iss.sleepers.push(iss.current_task);

      if (auto const* top_tcb = iss.sleepers.top()) {
         iss.next_wake_tick.store(top_tcb->wake_tick);
      }

      rtk_request_reschedule();
      port_yield();
   }

   void Scheduler::preempt_disable() { port_preempt_disable(); iss.preempt_disabled.store(true, std::memory_order_release); }
   void Scheduler::preempt_enable()  { iss.preempt_disabled.store(false, std::memory_order_release); port_preempt_enable(); }

   static void thread_trampoline(void* arg_void)
   {
      auto tcb = static_cast<TaskControlBlock*>(arg_void);
      tcb->entry_fn(tcb->arg);
      // If user function returns, park/surrender forever (could signal joiners later)
      while (true) Scheduler::yield();
   }


   struct StackLayout
   {
      TaskControlBlock* tcb;
      void* tls_base;
      std::size_t tls_size;
      void* stack_base;
      std::size_t stack_size;

      StackLayout(void* buffer_base, std::size_t buffer_size)
      {
         auto base = reinterpret_cast<std::uintptr_t>(buffer_base);
         auto end  = base + buffer_size;

         auto tcb_start = align_down(end - sizeof(TaskControlBlock), alignof(TaskControlBlock));
         tcb = reinterpret_cast<TaskControlBlock*>(tcb_start);

         std::uintptr_t tls_top = tcb_start;
         tls_size = 0;  // TODO: wire real TLS size later
         tls_base = reinterpret_cast<void*>(tls_top);

         auto stack_top = reinterpret_cast<std::uintptr_t>(tls_base);
         stack_base = reinterpret_cast<void*>(stack_top);
         stack_size = stack_top - base;

         assert(stack_size > 64 && "Buffer too small after carving TCB/TLS");
      }
   };


   Thread::Thread(EntryFunction fn, void* arg, void* stack_base, std::size_t stack_size, uint8_t priority)
   {
      assert(priority < MAX_PRIORITIES);

      StackLayout stack_layout(stack_base, stack_size);
      tcb = ::new (stack_layout.tcb) TaskControlBlock{
         .priority = priority,
         .stack_base = stack_base,
         .stack_size = stack_size,
         .entry_fn = fn,
         .arg = arg,
      };

      port_context_init(tcb->context(), stack_base, stack_size, thread_trampoline, tcb);

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

   std::size_t Thread::reserved_stack_size()
   {
      auto tcb_size = align_up(sizeof(TaskControlBlock), alignof(TaskControlBlock));
      auto tls_size = 0; // TODO
      return tcb_size + tls_size;
   }

   extern "C" void rtk_on_tick(void)
   {
      auto const now = Scheduler::tick_now();
      // Request reschedule if a wakeup is due
      if (now.has_reached(iss.next_wake_tick.load()) ||
          now.has_reached(iss.next_slice_tick.load()))
      {
         rtk_request_reschedule();
      }
   }

   extern "C" void rtk_request_reschedule(void)
   {
      iss.need_reschedule.store(true, std::memory_order_relaxed);
      // Under simulation we should return to the scheduler loop in user context
      // On bare metal, we should pend a software interrupt and return from this ISR
   }


   static void DEBUG_DUMP_READY_QUEUE(const char* where)
   {
   #if defined(RTK_SIMULATION)
      std::printf("[%u %s] bitmap: %s", port_tick_now(), where, iss.ready_matrix.bitmap_view().to_string().c_str());
      for (unsigned p = 0; p < MAX_PRIORITIES; ++p) {
         if (!iss.ready_matrix.empty_at(p)) {
            std::printf(" p%u(%zu)", p, iss.ready_matrix.size_at(p));
         }
      }
      std::printf(" current_task=%p\n", (void*)iss.current_task);
   #endif
   }

} // namespace rtk
