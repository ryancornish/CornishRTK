/**
 * @file port_traits.h
 * @brief Port-specific compile-time traits.
 *
 * Each port must provide exactly one `port_traits.h` defining the constants
 * below. This header:
 *  - must not include any other headers
 *  - must contain only macros
 *
 * The kernel relies on these values at compile time. The port implementation
 * must statically verify that its actual types and behaviour match them.
 */

#ifndef CYROS_PORT_TRAITS_H
#define CYROS_PORT_TRAITS_H

/**
 * @def CYROS_PORT_CONTEXT_SIZE
 * @brief Size of `port_context_t` in bytes.
 */
#define CYROS_PORT_CONTEXT_SIZE  40

/**
 * @def CYROS_PORT_CONTEXT_ALIGN
 * @brief Alignment requirement of `port_context_t` in bytes.
 *
 * Must be a power of two.
 */
#define CYROS_PORT_CONTEXT_ALIGN 8

/**
 * @def CYROS_PORT_STACK_ALIGN
 * @brief Required alignment of all thread stacks in bytes.
 *
 * Must be a power of two.
 */
#define CYROS_PORT_STACK_ALIGN 16

/**
 * @def CYROS_PORT_CACHE_LINE
 * @brief Cache line size in bytes.
 *
 * Used for false-sharing avoidance.
 */
#define CYROS_PORT_CACHE_LINE 64

/**
 * @def CYROS_PORT_CORE_COUNT
 * @brief Number of cores supported by this port.
 */
#define CYROS_PORT_CORE_COUNT 4

/**
 * @def CYROS_PORT_CAPTURE_LOCATION
 * @brief Capture file and line information on assert.
 *
 * Turning this on can increase code-size as the compiler attaches
 * strings to every CORTOS_ASSERT* location.
 */
#define CYROS_PORT_CAPTURE_LOCATION 1

#endif /* CYROS_PORT_TRAITS_H */
