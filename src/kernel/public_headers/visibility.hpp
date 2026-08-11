/**
 * @file visibility.hpp
 * @brief Marks declarations that form the public ABI of libcyros.
 *
 * Only relevant to the `lto_merged` archive strategy (see gcc-release.toml).
 * That strategy merges every object into one relocatable and then runs
 * `objcopy --localize-hidden`, so any symbol the compiler marked hidden becomes
 * local and cannot be referenced from outside the archive. The release toolchain
 * compiles with `-fvisibility=hidden`, which hides everything by default, so a
 * symbol stays reachable only if its declaration carries CYROS_PUBLIC.
 *
 * The rule is simply "everything declared in a public_headers file is public".
 * If you add a public entry point, annotate it. Forgetting produces a loud
 * undefined reference when something links against a release build.
 */

#ifndef CYROS_VISIBILITY_HPP
#define CYROS_VISIBILITY_HPP

#if defined(__GNUC__) || defined(__clang__)
#  define CYROS_PUBLIC __attribute__((visibility("default")))
#else
#  define CYROS_PUBLIC
#endif

#endif  // CYROS_VISIBILITY_HPP
