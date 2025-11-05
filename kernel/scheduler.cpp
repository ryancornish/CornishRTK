/**
 *
 */
#include <bitset>
#include <cassert>
#include <cornishrtk.hpp>

#include <atomic>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <deque>
#include <array>
#include <bit>
#include <time.h>

namespace rtk
{
   static void DEBUG_DUMP_READY_QUEUE(const char* where);

   struct TaskControlBlock
   {
      enum class State : uint8_t {
         Ready,
         Running,
         Sleeping,
         Blocked,
      } state{State::Ready};
      uint8_t priority{0};
      uint32_t timeslice_left{0};
      uint32_t wake_tick{0};

      port_context_t* context{nullptr};
      void*  stack_base{nullptr};
      size_t stack_size{0};
      Thread::EntryFunction entry_fn;
      void*  arg{nullptr};
   };

   struct InternalSchedulerState
   {
      std::atomic<bool> preempt_disabled{false};
      std::atomic<bool> need_reschedule{false};
      TaskControlBlock* current_task{nullptr};

      std::array<std::deque<TaskControlBlock*>, MAX_PRIORITIES> ready{}; // TODO: replace deque
      std::bitset<MAX_PRIORITIES> ready_bitmap;
      std::deque<TaskControlBlock*> sleep_queue{}; // TODO: replace with wheel/heap later
      std::atomic<uint32_t> next_wake_tick{UINT32_MAX};
      std::atomic<bool> slice_tick_pending{false};

      void reset() // Should I just do a self assignment from default ctor instead?
      {
         preempt_disabled.store(false);
         need_reschedule.store(false);
         current_task = nullptr;
         next_wake_tick.store(UINT32_MAX);
         slice_tick_pending.store(false);
      }
   };
   static /*constinit*/ InternalSchedulerState iss;

   static TaskControlBlock* pick_highest_ready()
   {
      if (iss.ready_bitmap.none()) return nullptr;
      uint8_t priority = std::countr_zero(iss.ready_bitmap.to_ulong());
      return iss.ready[priority].front();
   }

   static void remove_ready_head(uint8_t priority)
   {
      auto& queue = iss.ready[priority];
      queue.pop_front();
      if (queue.empty()) iss.ready_bitmap.reset(priority);
   }

   static void set_ready(TaskControlBlock* tcb)
   {
      tcb->state = TaskControlBlock::State::Ready;
      iss.ready[tcb->priority].push_back(tcb);
      iss.ready_bitmap.set(tcb->priority);
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
      port_switch(previous_task ? &previous_task->context : nullptr, iss.current_task->context);
   }

   static void schedule()
   {
      if (iss.preempt_disabled.load()) return;
      if (!iss.need_reschedule.exchange(false)) return;
      DEBUG_DUMP_READY_QUEUE("schedule() need_reschedule -> true");

      uint32_t const now = Scheduler::tick_now();
      uint32_t next_due = UINT32_MAX;
      for (auto it = iss.sleep_queue.begin(); it != iss.sleep_queue.end(); ) {
         TaskControlBlock* tcb = *it;
         if (static_cast<int32_t>(now - tcb->wake_tick) >= 0) {
            it = iss.sleep_queue.erase(it);
            set_ready(tcb);
         } else {
            next_due = std::min(next_due, tcb->wake_tick);
            ++it;
         }
      }
      iss.next_wake_tick.store(next_due, std::memory_order_relaxed);

      // Round robin rotation
      if (iss.slice_tick_pending.exchange(false, std::memory_order_acq_rel) &&
         iss.current_task &&
         iss.current_task->state == TaskControlBlock::State::Running)
      {
         if (iss.current_task->timeslice_left) --iss.current_task->timeslice_left;
         if (iss.current_task->timeslice_left == 0) {
            set_ready(iss.current_task);
            iss.current_task->state = TaskControlBlock::State::Ready;
            iss.current_task->timeslice_left = TIME_SLICE;
         }
      }

      auto* next_task = pick_highest_ready();
      if (!next_task) return; // Shhhh everyone is sleeping!
      remove_ready_head(next_task->priority);
      // Serve a fresh slice
      if (next_task->timeslice_left == 0) next_task->timeslice_left = TIME_SLICE;
      context_switch_to(next_task);
   }

   void Scheduler::init(uint32_t tick_hz)
   {
      iss.reset();
      port_init(tick_hz);
   }

   void Scheduler::start()
   {
      DEBUG_DUMP_READY_QUEUE("start() entry");
      auto first = pick_highest_ready();
      assert(first != nullptr);
      remove_ready_head(first->priority);
      DEBUG_DUMP_READY_QUEUE("start() head picked/removed");
      iss.current_task = first;
      iss.current_task->state = TaskControlBlock::State::Running;
      iss.current_task->timeslice_left = TIME_SLICE;

      port_start_first(iss.current_task->context);

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
         set_ready(iss.current_task); // put current at tail
         iss.current_task->state = TaskControlBlock::State::Ready;
      }
      rtk_request_reschedule();
      port_yield();
   }

   uint32_t Scheduler::tick_now()
   {
      return port_tick_now();
   }

   void Scheduler::sleep_for(uint32_t ticks)
   {
      if (ticks == 0) { yield(); return; }
      // Put current to sleep
      assert(iss.current_task && "no current thread");
      iss.current_task->state = TaskControlBlock::State::Sleeping;
      iss.current_task->wake_tick = tick_now() + ticks;
      iss.sleep_queue.push_back(iss.current_task);

      auto cur = iss.next_wake_tick.load(std::memory_order_relaxed);
      if (static_cast<int32_t>(iss.current_task->wake_tick - cur) < 0) {
         iss.next_wake_tick.store(iss.current_task->wake_tick, std::memory_order_relaxed);
      }

      rtk_request_reschedule();
      port_yield();
   }

   void Scheduler::preempt_disable() { port_preempt_disable(); }
   void Scheduler::preempt_enable()  { port_preempt_enable();  }

   static port_context_t* alloc_port_context()
   {
      size_t const size      = port_context_size();
      size_t const alignment = port_stack_align();
      size_t const allocate  = ((size + alignment - 1) / alignment) * alignment;

      return static_cast<port_context_t*>(std::aligned_alloc(alignment, allocate));
   }
   static void free_port_context(port_context_t* port_context_handle)
   {
      free(port_context_handle);
   }

   static void thread_trampoline(void* arg_void)
   {
      auto tcb = static_cast<TaskControlBlock*>(arg_void);
      tcb->entry_fn(tcb->arg);
      // If user function returns, park/surrender forever (could signal joiners later)
      for (;;) { Scheduler::yield(); }
   }

   Thread::Thread(EntryFunction fn, void* arg, void* stack_base, std::size_t stack_size, uint8_t priority)
   {
      assert(priority < MAX_PRIORITIES);
      tcb = new TaskControlBlock{
         .priority = priority,
         .context = alloc_port_context(),
         .stack_base = stack_base,
         .stack_size = stack_size,
         .entry_fn = fn,
         .arg = arg,
      };

      port_context_init(tcb->context, stack_base, stack_size, thread_trampoline, tcb);

      set_ready(tcb);
      DEBUG_DUMP_READY_QUEUE("Thread() created");
   }

   Thread::~Thread() {
      // Not handling live-thread destruction yet
      if (!tcb) return;
      free_port_context(tcb->context);
      delete tcb;
   }

   extern "C" void rtk_on_tick(void)
   {
      // Request reschedule if a wakeup is due
      if (static_cast<int32_t>(Scheduler::tick_now() - iss.next_wake_tick.load(std::memory_order_relaxed)) >= 0) {
         iss.need_reschedule.store(true, std::memory_order_relaxed);
      }

      // Flag that slice might have expired
      iss.slice_tick_pending.store(true, std::memory_order_relaxed);
   }

   extern "C" void rtk_request_reschedule(void)
   {
      iss.need_reschedule.store(1, std::memory_order_relaxed);
      // Under simulation we should return to the scheduler loop in user context
      // On bare metal, we should pend a software interrupt and return from this ISR
   }


static void DEBUG_DUMP_READY_QUEUE(const char* where)
{
#if defined(RTK_SIMULATION)
   std::printf("[READY %s] bitmap: %s", where, iss.ready_bitmap.to_string().c_str());
   for (unsigned p = 0; p < MAX_PRIORITIES; ++p) {
      if (!iss.ready[p].empty()) std::printf(" p%u(%zu)", p, iss.ready[p].size());
   }
   std::printf(" current_task=%p\n", (void*)iss.current_task);
#endif
}

} // namespace rtk
