/* SPDX-License-Identifier: LGPL-2.1-or-later */
/* Copyright (C) 2026 Ryan Cornish
 *
 * sigctx_intercept - hand control to user code when a chosen signal lands.
 *
 * Installs a handler for a caller-chosen signal. When the signal arrives, it captures
 * the interrupted context (registers + FP) and calls a caller-supplied 'handler'
 * callback. The handler is given the just-paused context and returns a context to run.
 * The interceptor then resumes whichever context the handler returned.
 *
 * The caller supplies everything through the config: the signal number, the stack the
 * handler runs on (handler_sp, handler_ss), the handler callback, and an opaque user
 * pointer passed through to it. The API makes no assumption about what the handler does
 * with the paused context. Returning a different context turns this into a context
 * switch, which is the common use, but returning the same context makes it a pure
 * inspection point, and anything in between is the caller's policy. Scheduling order,
 * how contexts are created, and whether this is a scheduler at all are not decided
 * here.
 *
 * The handler runs as ordinary code on its own stack with the trigger signal masked, so
 * it is free of async-signal-safety constraints and cannot be re-entered.
 *
 * The trigger signal is held blocked for the whole interception. block_extra extends
 * that to a caller-chosen set of additional signals, for a caller with a separate timer
 * or IPI signal that must not nest into the capture or the handler.
 *
 * Per-OS-thread: sigaltstack is per-OS-thread, so call sigctx_intercept_install once on
 * each OS thread that should be interceptible. The sigaction itself is process-wide.
 * Requires gcc or clang with GNU C extensions (the -std=gnu11 through -std=gnu23
 * dialects). x86-64 Linux, glibc >= 2.34. Compiles as C or C++.
 */
#ifndef SIGCTX_INTERCEPT_H
#define SIGCTX_INTERCEPT_H

#include <stddef.h>
#include <stdint.h>
#include <signal.h>

#include <sigctx/sigctx.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 * Called with the captured context of whatever was running when the signal landed.
 * Returns the context to resume, which must be non-null and must outlive the resume.
 * arg is the opaque pointer supplied in the config, passed through unchanged.
 */
typedef sigctx_ucontext_t* (*sigctx_handler_fn)(sigctx_ucontext_t* paused, void* arg);

typedef struct
{
   int               signo;       /* signal that triggers a switch, e.g. SIGURG/SIGUSR1 */
   uint8_t*          altstack_sp; /* base of the signal-delivery altstack, see below */
   size_t            altstack_ss; /* its size, see sigctx_altstack_min() below */
   uint8_t*          handler_sp;  /* base of the stack the handler and capture run on */
   size_t            handler_ss;  /* its size, see SIGCTX_INTERCEPT_MIN_FRAME below */
   sigctx_handler_fn handler;     /* picks the context to resume */
   void*             arg;         /* opaque, passed through to handler */
   sigset_t const*   block_extra; /* optional extra signals to hold blocked for the duration of an interception. NULL for none. */
} sigctx_intercept_cfg;

/*
 * What is the altstack? What is the handler stack? How are they different? How do I size them?
 *
 * altstack_sp/ss - where the LINUX KERNEL lays each raw signal frame (registers + FP)
 *   on delivery, via SA_ONSTACK. sigctx never writes here, it only hands the
 *   region to sigaltstack(). Size it with sigctx_altstack_min(depth):
 *
 *     - depth is 1 for a lone interceptor that no other signal preempts.
 *     - depth is your NESTING count when other signals are installed (each its own
 *       plain SA_ONSTACK handler) and can preempt one another and this interceptor.
 *       Under SA_ONSTACK the kernel stacks a nested frame directly above the
 *       current one and pops it on sigreturn, so at peak, one frame per
 *       simultaneously-live handler coexists here.
 *
 *       Note that THIS interceptor occupies a slot only during its capture
 *       phase, not while your handler callback runs. The capture diverts
 *       execution onto handler_sp before calling you, and the kernel picks the
 *       altstack base rather than nesting whenever the interrupted SP is outside
 *       the altstack. So a signal arriving during your callback starts a fresh
 *       frame at the base. Count the capture window, not the callback, or you
 *       will over-size by one level per interceptor. The peak equals the number
 *       of distinct priority levels in your signal mask table, which stays
 *       finite only if that table is an ACYCLIC ordering (level N preemptible
 *       strictly by levels above N). Count the levels, pass that as depth.
 *
 * handler_sp/ss - where sigctx relocates the captured context and runs your
 *   handler callback, AFTER capture, as ordinary code. Size it with
 *   SIGCTX_INTERCEPT_MIN_FRAME (or larger for a deep handler call chain). This
 *   stack does not nest: the trigger signal is masked while the handler runs.
 *
 * The two are separate because they serve different phases (kernel delivery vs
 * post-capture handler) and have different sizing laws (nesting vs not). One
 * altstack is shared across every SA_ONSTACK signal on the OS-thread because
 * sigaltstack registers exactly one per OS-thread. Which is why its sizing must
 * account for all of them nesting, while each interceptor's handler stack is
 * private to that interceptor.
 */

/*
 * Minimum bytes for altstack_ss. sigctx_altstack_slot_min() is one runtime
 * signal frame plus the handler-frame margin, measured against THIS machine's
 * _SC_SIGSTKSZ (so it is correct on AVX-512/AMX without a compile-time guess).
 * sigctx_altstack_min(depth) is that times the nesting depth. These are NOT
 *  compile-time constants. Call them at startup to size a caller-owned buffer.
 * Install re-checks and returns -ENOMEM if altstack_ss is below one slot.
 */
size_t sigctx_altstack_slot_min(void);
size_t sigctx_altstack_min(unsigned depth);

/*
 * Conservative compile-time lower bound for handler_ss, derived from the inline FP
 * capacity. Use it to size a handler stack at compile time. The authoritative check
 * is at runtime inside sigctx_intercept_install, which measures this machine's actual
 * XSAVE extent with sigctx_fpstate_size() and returns -ERANGE if handler_ss is too
 * small. Whenever SIGCTX_FPSTATE_CAPACITY covers the runtime size, a stack sized by
 * this macro clears that check with room to spare.
 */
#define SIGCTX_INTERCEPT_MIN_FRAME (sizeof(sigctx_ucontext_t) + SIGCTX_FPSTATE_CAPACITY + 4096u)

/*
 * Install the interceptor on the calling thread. A malformed config (null cfg,
 * null handler, null handler_sp, null altstack_sp, or signo <= 0) is a caller
 * bug and asserts. Returns 0 on success, or a negative errno for a genuine
 * runtime condition the caller cannot precompute:
 *
 *   -ENOMEM  altstack_ss is below one runtime signal frame. FIX: size the
 *            altstack with sigctx_altstack_min(depth), where depth is your
 *            nesting-level count (see the config doc above).
 *   -ERANGE  handler_ss is below the runtime capture frame. FIX: raise it to at
 *            least SIGCTX_INTERCEPT_MIN_FRAME; on an AVX-512/AMX machine also
 *            raise SIGCTX_FPSTATE_CAPACITY (or use a dynamically sized handler
 *            stack against sigctx_fpstate_size()).
 *   other    a sigaltstack or sigaction errno, passed through unchanged.
 *
 * Build with -DSIGCTX_DIAGNOSTICS to have a failing install print the offending
 * sizes and the fix to stderr, which turns a bare errno at the call site into an
 * actionable message. The caller decides how to handle failure.
 */
int sigctx_intercept_install(sigctx_intercept_cfg const* cfg);

#ifdef __cplusplus
}
#endif

#endif /* SIGCTX_INTERCEPT_H */
