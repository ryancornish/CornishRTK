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
 * Merging is what makes this matter: a normal archive lets the consumer's linker
 * pull individual members, but a single merged member is all-or-nothing, so
 * without hiding every internal symbol lands in every consumer link and can
 * collide.
 *
 * The rule is simply "everything declared in a public_headers file is public".
 * If you add a public entry point, annotate it. Forgetting produces a loud
 * undefined reference when something links against a release build, never
 * silently wrong behaviour.
 *
 * Debug and unit-test toolchains do not pass -fvisibility=hidden, so the macro
 * is inert there and costs nothing.
 */

#ifndef CYROS_VISIBILITY_HPP
#define CYROS_VISIBILITY_HPP

#if defined(__GNUC__) || defined(__clang__)
#  define CYROS_PUBLIC __attribute__((visibility("default")))
#else
#  define CYROS_PUBLIC
#endif

#endif  // CYROS_VISIBILITY_HPP
