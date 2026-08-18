/**
 * @file guarded_stack.hpp
 * @brief Thread stacks backed by mmap with a guard page, for unit tests.
 *
 * Shared by every unit test. The unit test root is on the include path, so use
 * the rooted form from any depth:
 *
 *    #include <common/guarded_stack.hpp>
 */

#ifndef CYROS_TEST_GUARDED_STACK_HPP
#define CYROS_TEST_GUARDED_STACK_HPP

#include <cyros/kernel/thread.hpp>
#include <cyros/port/port_traits.h>

#include <sys/mman.h>
#include <unistd.h>

#include <cstddef>
#include <cstdio>
#include <cstdlib>
#include <span>
#include <utility>

namespace cyros::test
{

/**
 * @brief A thread stack with a PROT_NONE guard page beneath it.
 *
 * Cyros stacks grow DOWN and the TCB is carved from the TOP of the buffer, so an
 * overflow runs off the LOW end. The guard sits exactly there, which turns an
 * overrun into a SIGSEGV at the instruction that caused it.
 *
 * @example
 * @code
 *    guarded_stack s;
 *    thread t(fn, s, thread::priority(0), core0);
 * @endcode
 *
 * Some tests deliberately keep static buffers: the port must work with a plain
 * caller-owned array, which is what a real target provides, and only a test that
 * uses one proves it still does.
 */
class guarded_stack
{
public:
   // Matches the static-buffer tests, so swapping one for the other retunes nothing.
   static constexpr std::size_t default_bytes = thread::min_stack_size + (16 * 1024);

   explicit guarded_stack(std::size_t bytes = default_bytes)
   {
      std::size_t const page = page_size();

      usable_        = round_up(bytes < thread::min_stack_size ? thread::min_stack_size : bytes, page);
      mapping_bytes_ = page + usable_;

      mapping_ = mmap(nullptr, mapping_bytes_, PROT_READ | PROT_WRITE,
                      MAP_PRIVATE | MAP_ANONYMOUS | MAP_STACK, -1, 0);
      if (mapping_ == MAP_FAILED) die("mmap");

      // The guard is the lowest page, so it is what a downward overrun hits.
      if (mprotect(mapping_, page, PROT_NONE) != 0) die("mprotect");

      base_ = static_cast<std::byte*>(mapping_) + page;
   }

   ~guarded_stack() { release(); }

   guarded_stack(guarded_stack&& other) noexcept
      : mapping_(std::exchange(other.mapping_, nullptr)),
        mapping_bytes_(std::exchange(other.mapping_bytes_, 0)),
        base_(std::exchange(other.base_, nullptr)),
        usable_(std::exchange(other.usable_, 0))
   {}

   guarded_stack& operator=(guarded_stack&& other) noexcept
   {
      if (this != &other) {
         release();
         mapping_       = std::exchange(other.mapping_, nullptr);
         mapping_bytes_ = std::exchange(other.mapping_bytes_, 0);
         base_          = std::exchange(other.base_, nullptr);
         usable_        = std::exchange(other.usable_, 0);
      }
      return *this;
   }

   guarded_stack(guarded_stack const&)            = delete;
   guarded_stack& operator=(guarded_stack const&) = delete;

   [[nodiscard]] std::span<std::byte> span() const noexcept { return {base_, usable_}; }

   /// Intentionally implicit: a guarded_stack is meant to be passed where a buffer goes.
   operator std::span<std::byte>() const noexcept { return span(); }

private:
   void release() noexcept
   {
      if (mapping_ != nullptr) munmap(mapping_, mapping_bytes_);
      mapping_       = nullptr;
      mapping_bytes_ = 0;
      base_          = nullptr;
      usable_        = 0;
   }

   [[noreturn]] static void die(char const* what)
   {
      std::fprintf(stderr, "guarded_stack: %s failed\n", what);
      std::abort();
   }

   static std::size_t page_size()
   {
      auto const n = sysconf(_SC_PAGESIZE);
      return n > 0 ? static_cast<std::size_t>(n) : 4096u;
   }

   static std::size_t round_up(std::size_t v, std::size_t to)
   {
      return ((v + to - 1) / to) * to;
   }

   void*       mapping_{nullptr};
   std::size_t mapping_bytes_{0};
   std::byte*  base_{nullptr};
   std::size_t usable_{0};
};

static_assert(CYROS_PORT_STACK_ALIGN <= 4096, "a page-aligned mapping must satisfy the port's stack alignment");

} // namespace cyros::test

#endif // CYROS_TEST_GUARDED_STACK_HPP
