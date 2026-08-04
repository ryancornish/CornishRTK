/**
 * @file port_linux_common.hpp
 * @brief Contract shared by the linux ports (coop and preempt).
 *
 * NOT a port. This directory holds the parts of a linux port that do not depend
 * on how the port switches contexts, so each real port under src/port/linux/
 * compiles it alongside its own sources rather than linking a separate library.
 * Each port therefore gets its own object built with its own traits and flags,
 * which is what lets CYROS_PORT_DEBUG_MODE and friends stay per-port.
 *
 * WHAT BELONGS HERE
 * -----------------
 * Code that is identical between coop and preempt because it is about Linux or
 * the host CPU, not about scheduling: panic reporting, debugger hooks, CPU hints,
 * and plain per-thread storage. If a candidate differs between the two ports for
 * any reason other than a stray comment, it does NOT belong here.
 *
 * WHAT DOES NOT
 * -------------
 * Anything touching a port's own per-core state, context switching, signal
 * masking, or its time driver. Those are the actual differences between the two
 * ports and duplicating their shapes here would couple them.
 *
 * The port contract functions defined in port_linux_common.cpp are declared in
 * <cyros/port/port.h>, not here, because they are part of the kernel-facing C API
 * and must keep C linkage. Only the C++ internals appear below. A port that
 * includes this file must NOT also define the contract functions.
 */

#ifndef CYROS_PORT_LINUX_COMMON_HPP
#define CYROS_PORT_LINUX_COMMON_HPP

namespace cyros::port
{

/**
 * @brief Print the source lines surrounding a panic location.
 *
 * Reads the file off disk at panic time, so it is only useful in a build whose
 * source tree is still present. Best effort: prints nothing if the file cannot be
 * opened. Used by cyros_port_system_error().
 *
 * @param file        Path as captured by CYROS_PORT_CAPTURE_FILE, may be empty.
 * @param target_line Line to highlight.
 * @param range       Lines of context either side.
 */
void print_formatted_context(char const* file, int target_line, int range = 2);

} // namespace cyros::port

#endif /* CYROS_PORT_LINUX_COMMON_HPP */
