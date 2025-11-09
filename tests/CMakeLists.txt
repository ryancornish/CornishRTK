# tests/CMakeLists.txt

# Simple demo that uses the kernel API
add_executable(rtk_two_thread_demo
  rtk_two_thread_demo.cpp
)
target_link_libraries(rtk_two_thread_demo
  PRIVATE cornishrtk_kernel     # brings rtk::port transitively (PUBLIC)
)

# If you want a test that calls port APIs directly, link rtk::port explicitly:
add_executable(two_thread_demo
  two_thread_demo.cpp
)
target_link_libraries(two_thread_demo
  PRIVATE cornishrtk_kernel
          rtk::port
)
