#include <cyros/config/config.hpp>
#include <cyros/port/port_traits.h>

#include <cstdint>
#include <limits>

namespace cyros
{

/* ============================================================================
 * Configuration validation
 * ========================================================================= */
static_assert(1 <= config::cores && config::cores <= CYROS_PORT_CORE_COUNT,
              "Port does not support configured amount of cores.");
static_assert(config::max_priorities < std::numeric_limits<uint32_t>::digits,
              "Priorities unsupported by kernel implementation."
              "See 'scheduler::tie_rotor' and 'thread_ready_matrix::bitmap'");

} // namespace cyros
