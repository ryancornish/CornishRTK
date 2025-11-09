
#ifndef _PORT_H_
#define _PORT_H_
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct port_context port_context_t;
typedef void (*port_entry_t)(void*);

/* Boot & time */
void     port_init(uint32_t tick_hz);   /* set up periodic tick (0 => default) */
uint32_t port_tick_now(void);           /* monotonic tick */

/* Context lifecycle */
void port_context_init(port_context_t* ctx,
                       void* stack_base, size_t stack_size,
                       port_entry_t entry, void* arg);

// Can't apply '__attribute__((noreturn))' because Boost.Context port actually does return from this
void port_start_first(port_context_t* first);

void port_yield(void);                        /* thread asks to reschedule */
void port_switch(port_context_t** from, port_context_t* to);

/* ISR/Preempt model */
void port_isr_enter(void);
void port_isr_exit(int request_switch);       /* nonzero => resched */
void port_preempt_disable(void);
void port_preempt_enable(void);

/* Optional helpers */
void   port_irq_disable(void);
void   port_irq_enable(void);

size_t port_context_size(void);               /* sizeof(port_context_t) */
size_t port_context_align(void);
size_t port_stack_align(void);                /* e.g., 16 on x86-64 */

/* What the port wants to do during the idle thread. I.e. power saving */
void port_idle(void);

// Kernel implements these!
void rtk_on_tick(void);                 // tick ISR hook
void rtk_request_reschedule(void);      // ask kernel to reschedule

#ifdef __cplusplus
}
#endif

#endif
