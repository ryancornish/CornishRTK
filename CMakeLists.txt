cmake_minimum_required(VERSION 3.16)

project(CornishRTK LANGUAGES C CXX)

# Global settings
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Public headers
include_directories(${CMAKE_SOURCE_DIR}/include)

# Port backend
set(RTK_PORT "linux/boost_context" CACHE STRING "CornishRTK backend port")
set_property(CACHE RTK_PORT PROPERTY STRINGS linux linux/boost_context)

# Build selected port (must provide target rtk_port and alias rtk::port)
add_subdirectory(port/${RTK_PORT})

# Kernel & tests
add_subdirectory(kernel)
add_subdirectory(tests)
