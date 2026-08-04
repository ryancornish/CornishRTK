#ifndef CYROS_CONFIG_HPP
#define CYROS_CONFIG_HPP

#include <cstdint>

namespace cyros::config
{

inline constexpr std::size_t    cores          = 4;
inline constexpr std::size_t    max_wait_nodes = 8;
inline constexpr std::size_t    max_priorities = 31;

}  // namespace cyros::config

#endif // CYROS_CONFIG_HPP