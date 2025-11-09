# port/linux/CMakeLists.txt
find_package(Boost REQUIRED COMPONENTS context)

# Config interface: carries ONLY the macros (and optional includes)
add_library(rtk_port_config INTERFACE)
target_compile_definitions(rtk_port_config INTERFACE
  RTK_SIZEOF_PORT_CONTEXT_T=56
  RTK_ALIGNOF_PORT_CONTEXT_T=8
)
# If the public headers need to see the macros, keep the shared include dir:
target_include_directories(rtk_port_config INTERFACE
  ${CMAKE_SOURCE_DIR}/include
)

# Port implementation TU(s). Needs the macros to compile itself
add_library(cornishrtk_port_impl STATIC
  port_linux_boost_continuation.cpp
)
target_compile_features(cornishrtk_port_impl PUBLIC cxx_std_17)
target_compile_definitions(cornishrtk_port_impl PUBLIC
  RTK_SIMULATION=1
)
target_include_directories(cornishrtk_port_impl PUBLIC
  ${Boost_INCLUDE_DIRS}
  ${CMAKE_SOURCE_DIR}/include
)
target_link_libraries(cornishrtk_port_impl PUBLIC
  Boost::context
  rtk_port_config       # Pulls macros into impl compile lines
)

# Consumer interface. Users link this single target to get everything
add_library(rtk_port_linux INTERFACE)
target_link_libraries(rtk_port_linux INTERFACE
  cornishrtk_port_impl  # symbols + Boost
  rtk_port_config       # macros (size/align) + any shared includes
)

# Stable alias used by the rest of the tree
add_library(rtk::port ALIAS rtk_port_linux)
