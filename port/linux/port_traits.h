/**
 * port_traits.h
 * Port traits for Linux Boost.Context port backend.
 */
#ifndef _PORT_TRAITS_H_
#define _PORT_TRAITS_H_

#define RTK_SIMULATION 1

#define RTK_PORT_CONTEXT_SIZE   56u   // must match sizeof(port_context)
#define RTK_PORT_CONTEXT_ALIGN  8u    // must match alignof(port_context)
#define RTK_STACK_ALIGN         16u   // initial SP alignment for this port
#define RTK_TLS_SIZE            0u    // TLS unmanaged in the Linux sim
#define RTK_TLS_ALIGN           alignof(std::max_align_t)

#endif
