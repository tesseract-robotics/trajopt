# Patch step of the OSQP ExternalProject: OSQP 1.0.0 prints "Polishing not needed - no active set detected at optimal
# point" whenever polishing finds no active constraint, whatever settings->verbose says (osqp/osqp#785). Guard it as
# osqp/osqp#795 does; drop this step once an OSQP release carries that fix.
#
# Usage: cmake -DOSQP_SOURCE_DIR=<osqp source> -P guard_polish_message.cmake
#
# Idempotent: an already guarded source is left as it is, so the step may run again.

if(NOT OSQP_SOURCE_DIR)
  message(FATAL_ERROR "OSQP_SOURCE_DIR is not set")
endif()

set(polish_c "${OSQP_SOURCE_DIR}/src/polish.c")
file(READ "${polish_c}" source)

set(unguarded [[    c_print("Polishing not needed - no active set detected at optimal point\n");]])
set(guarded
    [[#ifdef OSQP_ENABLE_PRINTING
    if (settings->verbose)
      c_print("Polishing not needed - no active set detected at optimal point\n");
#endif /* ifdef OSQP_ENABLE_PRINTING */]])

string(FIND "${source}" "${guarded}" guarded_at)
if(guarded_at GREATER -1)
  return()
endif()

string(FIND "${source}" "${unguarded}" unguarded_at)
if(unguarded_at EQUAL -1)
  message(FATAL_ERROR "${polish_c} has no unguarded polishing message: update or drop this patch step "
                      "for the fetched OSQP version")
endif()

string(
  REPLACE "${unguarded}"
          "${guarded}"
          source
          "${source}")
file(WRITE "${polish_c}" "${source}")
message(STATUS "Guarded the polishing message in ${polish_c} behind settings->verbose")
