/**
 * @file port_linux_common.cpp
 * @brief Port-contract functions identical across the linux ports.
 *
 * See port_linux_common.hpp for what belongs here and what does not. Compiled once
 * per port, so each port's traits apply to its own copy.
 */

#include "port_linux_common.hpp"

#include <cyros/port/port.h>

#include <csignal>
#include <cstdio>
#include <fstream>
#include <string>
#include <unistd.h>

namespace cyros::port
{

void print_formatted_context(char const* file, int target_line, int range)
{
   static constexpr auto CLR_RESET  = "\033[0m";
   static constexpr auto CLR_RED    = "\033[1;31m";
   static constexpr auto CLR_ORANGE = "\033[38;5;208m";

   std::ifstream fs(file);
   if (!fs.is_open()) return;

   std::string text;
   int current = 0;
   int start = (target_line - range > 0) ? target_line - range : 1;
   int end   = target_line + range;

   while (std::getline(fs, text)) {
      current++;
      if (current >= start && current <= end) {
         std::printf("├ ");
         std::printf("%s%4d%s  ", CLR_ORANGE, current, CLR_RESET);
         if (current == target_line) {
            std::printf("%s>> %s%s\n", CLR_RED, text.c_str(), CLR_RESET);
         } else {
            std::printf("   %s\n", text.c_str());
         }
      }
      if (current > end) break;
   }
}

} // namespace cyros::port


/* ----------------------------------------------------------------------------
 * Thread-Local Storage
 *
 * A plain per-OS-thread slot the kernel owns. It carries no port semantics, so it
 * lives here rather than inside either port's per-core state.
 * ------------------------------------------------------------------------- */

static thread_local void* g_tls_pointer = nullptr;

void cyros_port_set_tls_pointer(void* tls_base)
{
   g_tls_pointer = tls_base;
}

void* cyros_port_get_tls_pointer(void)
{
   return g_tls_pointer;
}


/* ----------------------------------------------------------------------------
 * CPU Hints
 *
 * Absent (located in specific linux port instead):
 * - cyros_port_idle()
 * ------------------------------------------------------------------------- */

void cyros_port_cpu_relax(void)
{
#if defined(__x86_64__) || defined(__i386__)
   __builtin_ia32_pause();
#elif defined(__aarch64__) || defined(__arm__)
   __asm__ __volatile__("yield");
#endif
}


/* ----------------------------------------------------------------------------
 * Debug & Diagnostics
 * ------------------------------------------------------------------------- */

void cyros_port_system_error(uintptr_t auxilary1, uintptr_t auxilary2, char const* file_optional, int line_optional)
{
   std::printf("KERNEL PANIC at %s:%d\n", file_optional, line_optional);
   cyros::port::print_formatted_context(file_optional, line_optional);
   std::printf("└ AUX1: 0x%lX, AUX2: 0x%lX\n", auxilary1, auxilary2);

   // Uncomment locally to park the faulting core so gdb can attach and inspect a
   // live crash. Off by default because it spins forever, which turns any panic
   // during an automated suite run into an indefinite hang with no output.
   // cyros_port_wait_for_debugger();

   // stdio is BLOCK buffered when stdout is a file or a pipe, and abort() does
   // not flush it. Without this a panic under `prog > log` prints nothing at all
   // and the log ends with a bare "terminate called without an active
   // exception", which is exactly as useless as it sounds. Flush everything
   // before dying.
   std::fflush(nullptr);

   std::terminate();
}

void cyros_port_wait_for_debugger(void)
{
   // Masked while parked so a reschedule cannot fire underneath the spin. On the
   // coop port the mask helpers are plain counters, so this costs nothing there.
   auto irq_mask = cyros_port_irq_save();

   volatile int pause = 1;
   std::printf("Attach GDB for PID: %d\n'set var pause = 0' to continue\n", getpid());
   std::fflush(nullptr); // or the invitation is still in the buffer while we spin

   while (pause) {
      usleep(1000);
   }
   cyros_port_irq_restore(irq_mask);
}

void cyros_port_breakpoint(void)
{
#if defined(__x86_64__) || defined(__i386__)
   __asm__ __volatile__("int3");
#elif defined(__aarch64__) || defined(__arm__)
   __builtin_trap();
#else
   raise(SIGTRAP);
#endif
}

void* cyros_port_get_stack_pointer(void)
{
   void* sp;
#if defined(__x86_64__)
   __asm__ __volatile__("mov %%rsp, %0" : "=r"(sp));
#elif defined(__i386__)
   __asm__ __volatile__("mov %%esp, %0" : "=r"(sp));
#elif defined(__aarch64__)
   __asm__ __volatile__("mov %0, sp" : "=r"(sp));
#elif defined(__arm__)
   __asm__ __volatile__("mov %0, sp" : "=r"(sp));
#else
   int dummy;
   sp = &dummy;
#endif
   return sp;
}
