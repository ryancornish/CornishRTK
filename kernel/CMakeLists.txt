# Kernel library
add_library(cornishrtk_kernel STATIC
  scheduler.cpp
)

# Kernel depends on the selected port.
target_link_libraries(cornishrtk_kernel PUBLIC rtk::port)

# Expose public headers
target_include_directories(cornishrtk_kernel PUBLIC
  ${CMAKE_SOURCE_DIR}/include
)

# TODO: needed?
target_compile_features(cornishrtk_kernel PUBLIC cxx_std_23)
