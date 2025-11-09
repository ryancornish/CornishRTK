# kernel/CMakeLists.txt
add_library(cornishrtk_kernel STATIC
  hooks_stub.cpp # Probably dont need this anymore
  scheduler.cpp
)

# Link the alias that bundles both the impl and the config macros
target_link_libraries(cornishrtk_kernel
  PUBLIC rtk::port
)

target_include_directories(cornishrtk_kernel PUBLIC
  ${CMAKE_SOURCE_DIR}/include
)
