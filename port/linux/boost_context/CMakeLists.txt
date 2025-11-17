find_package(Boost REQUIRED COMPONENTS context)

add_library(rtk_port STATIC
  port_linux_boost_context.cpp
)

# Include dirs:
#  - This directory, so kernel can #include <port_traits.h>
#  - Public include/ for <port.h>
#  - Boost headers
target_include_directories(rtk_port PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}     # contains port_traits.h
  ${CMAKE_SOURCE_DIR}/include     # contains port.h
  ${Boost_INCLUDE_DIRS}
)

# Link Boost.Context
target_link_libraries(rtk_port PUBLIC Boost::context)

# Target-level C++ standard (Linux port is C++ but exposes a C ABI)
target_compile_features(rtk_port PUBLIC cxx_std_23)

# Provide a stable alias used by kernel/tests
add_library(rtk::port ALIAS rtk_port)
