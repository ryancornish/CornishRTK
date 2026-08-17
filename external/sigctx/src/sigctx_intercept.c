/* SPDX-License-Identifier: LGPL-2.1-or-later */
/* Copyright (C) 2026 Ryan Cornish
 *
 * sigctx_intercept - implementation. Generic capture-and-handle over sigctx. Built
 * in the gnu11..gnu23 dialects.
 */

#include <sigctx/sigctx_intercept.h>

#include <assert.h>
#include <errno.h>
#include <signal.h>
#include <stdint.h>
#include <string.h>
#include <unistd.h>

/* Opt-in install diagnostics. The sizing returns (-ENOMEM, -ERANGE) are runtime
 * conditions a caller cannot precompute, so a bare errno leaves them guessing
 * which buffer and which macro to change. Define SIGCTX_DIAGNOSTICS to have a
 * failing install print the actual comparison and the fix to stderr. Off by
 * default: zero cost (and no stdio pulled in). */
#ifdef SIGCTX_DIAGNOSTICS
#include <stdio.h>
#define SIGCTX_DIAG(...) ((void)fprintf(stderr, "sigctx: " __VA_ARGS__))
#else
#define SIGCTX_DIAG(...) ((void)0)
#endif

/* Per-thread because sigaltstack and the active config are per-thread. */
_Thread_local static sigctx_intercept_cfg g_cfg;
_Thread_local static size_t g_fp_cap; /* bytes to reserve for a captured FP area */

/* Resolved copy of cfg.block_extra, empty if the caller passed none. Held by value so
 * the handler can consult it at delivery time without the caller's set having to
 * outlive install. */
_Thread_local static sigset_t g_block_extra;

/* OR the signals set in src into dst. Hand-rolled rather than sigorset so the library
 * needs no _GNU_SOURCE, and built only from the async-signal-safe sig* primitives so
 * it is safe to call from interceptor_on_signal. */
static void sigctx_mask_or(sigset_t* dst, sigset_t const* src)
{
   for (int s = 1; s < _NSIG; ++s) {
      if (sigismember(src, s) == 1) {
         sigaddset(dst, s);
      }
   }
}

__attribute__((noreturn))
static void interceptor_trampoline(sigctx_ucontext_t* paused_ctx)
{
   /* Ordinary code on the user-handler stack, trigger signal masked. The signal
    * handler has already returned, so we are no longer in async-signal context and
    * assert() is safe here (it is not safe inside interceptor_on_signal). */
   sigctx_ucontext_t* next_ctx = g_cfg.handler(paused_ctx, g_cfg.arg);
   assert(next_ctx); /* the handler contract requires a resumable context */
   sigctx_resume(next_ctx);
}

static void interceptor_on_signal(int sig, siginfo_t* info, void* opaque)
{
   (void)sig;
   (void)info;

   if (g_cfg.handler == NULL) return; // Ignore if threads config incomplete

   sigctx_ucontext_t* kctx = (sigctx_ucontext_t*)opaque;

   /* Carve a private workspace from the top of the handler stack. */
   uintptr_t sp = (uintptr_t)(g_cfg.handler_sp + g_cfg.handler_ss) & ~(uintptr_t)0xF;

   sp -= sizeof(sigctx_ucontext_t);
   sp &= ~(uintptr_t)0x3F;
   sigctx_ucontext_t* stored = (sigctx_ucontext_t*)sp;

   sp -= g_fp_cap;
   sp &= ~(uintptr_t)0x3F;
   uint8_t* fpbuf = (uint8_t*)sp;

   /* Deep-copy the kernel frame (registers + FP) into our own storage and repoint. */
   (void)sigctx_copy(stored, fpbuf, g_fp_cap, kctx);

   /* The captured context resumes preemptible. The diverted one must not be
    * re-interrupted while the user-handler runs on the shared handler stack. */
   sigdelset(&stored->uc_sigmask, g_cfg.signo);
   sigaddset(&kctx->uc_sigmask, g_cfg.signo);

   /* Hold the extra signals blocked through the handler phase too. sa_mask covered
    * only the capture, so without this they would re-open the moment the kernel
    * sigreturns into the trampoline, which is where the handler does its real work. */
   sigctx_mask_or(&kctx->uc_sigmask, &g_block_extra);

   /* Interceptor trampoline needs RSP == 8 (mod 16). We arrive via a jump, not a call. */
   sp = (sp & ~(uintptr_t)0xF) - 8;

   /* Divert the kernel's return into the interceptor trampoline on the handler stack. */
   kctx->uc_mcontext.rdi = (uint64_t)stored;
   kctx->uc_mcontext.rsp = (uint64_t)sp;
   kctx->uc_mcontext.rip = (uint64_t)interceptor_trampoline;
}

size_t sigctx_altstack_slot_min(void)
{
   long want = sysconf(_SC_SIGSTKSZ);
   size_t frame = (want > 0) ? (size_t)want : (size_t)SIGSTKSZ;

   /* The handler's own C frame runs on this stack above the kernel-laid signal
    * frame, so budget for it too, matching the +4096 in SIGCTX_INTERCEPT_MIN_FRAME. */
   return frame + 4096u;
}

size_t sigctx_altstack_min(unsigned depth)
{
   return (depth == 0 ? 1u : depth) * sigctx_altstack_slot_min();
}

int sigctx_intercept_install(sigctx_intercept_cfg const* cfg)
{
   /* Config is the caller's contract, so a malformed config asserts. The returns
    * below are for genuine runtime conditions the caller cannot check in advance:
    * the machine's signal-frame size, the supplied stacks' adequacy at the runtime
    * XSAVE size, and the underlying syscalls. */
   assert(cfg != NULL);
   assert(cfg->altstack_sp != NULL);
   assert(cfg->handler_sp != NULL);
   assert(cfg->handler != NULL);
   assert(cfg->signo > 0);

   size_t const slot_min = sigctx_altstack_slot_min();

   /* The altstack must hold at least one runtime signal frame, and for nesting
    * (any extra signals preempting one another and the bottom handler) it
    * must hold one per simultaneously-live frame. The caller declares the depth
    * it designed for via its mask priority table. Verified here the buffer covers it. */
   if (cfg->altstack_ss < slot_min) {
      SIGCTX_DIAG("altstack_ss %zu < one signal frame %zu: raise it to at least "
                  "sigctx_altstack_min(depth)\n", cfg->altstack_ss, slot_min);
      return -ENOMEM;
   }

   uint32_t xsave = sigctx_fpstate_size();
   if (xsave == 0) {
      xsave = (uint32_t)sizeof(struct sigctx_fpstate); /* no XSAVE: legacy floor */
   }

   /* The handler stack must hold the captured context, its FP area (sized to THIS CPU's
    * enabled XSAVE set at runtime, not a compile-time guess), and the handler's frame.
    * Checking the real size here catches an AVX-512/AMX machine with -ERANGE instead
    * of overflowing the handler stack at capture. This keeps the library independent of
    * SIGCTX_FPSTATE_CAPACITY, which is purely a consumer-side inline-struct concern. */
   size_t const need = sizeof(sigctx_ucontext_t) + (size_t)xsave + 4096u;
   if (cfg->handler_ss < need) {
      SIGCTX_DIAG("handler_ss %zu < required %zu: raise it to at least "
                  "SIGCTX_INTERCEPT_MIN_FRAME, and on AVX-512/AMX also raise "
                  "SIGCTX_FPSTATE_CAPACITY\n", cfg->handler_ss, need);
      return -ERANGE;
   }

   g_fp_cap = xsave;
   g_cfg = *cfg;

   /* Resolve the extra block set to a by-value copy the handler can read at delivery
    * time. NULL means none, which leaves an empty set and reproduces the prior
    * behaviour exactly. */
   if (cfg->block_extra != NULL) {
      g_block_extra = *cfg->block_extra;
   } else {
      sigemptyset(&g_block_extra);
   }

   stack_t ss;
   ss.ss_sp    = cfg->altstack_sp;
   ss.ss_size  = cfg->altstack_ss;
   ss.ss_flags = 0;
   if (sigaltstack(&ss, NULL) == -1) {
      return -errno;
   }

   struct sigaction sa;
   memset(&sa, 0, sizeof sa);
   sa.sa_sigaction = interceptor_on_signal;
   sigemptyset(&sa.sa_mask);
   sigctx_mask_or(&sa.sa_mask, &g_block_extra); /* block the extras during capture */
   sa.sa_flags = SA_ONSTACK | SA_RESTART | SA_SIGINFO;
   if (sigaction(cfg->signo, &sa, NULL) == -1) {
      return -errno;
   }
   return 0;
}
