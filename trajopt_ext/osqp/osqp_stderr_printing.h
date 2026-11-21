/*
 * OSQP custom printing header, passed to OSQP's build as OSQP_CUSTOM_PRINTING.
 *
 * OSQP's default c_print is printf, so its verbose log and its error messages (c_eprint and
 * _osqp_error, which print regardless of settings->verbose) go to stdout. A library has no
 * business writing to a process's stdout: an application that uses it as a data channel gets
 * corrupted output. This sends every OSQP message to stderr instead; what is printed, and
 * when, is unchanged.
 */
#ifndef TRAJOPT_OSQP_STDERR_PRINTING_H
#define TRAJOPT_OSQP_STDERR_PRINTING_H

#include <stdio.h>
#include <string.h>

#define c_print(...) fprintf(stderr, __VA_ARGS__)

#endif /* TRAJOPT_OSQP_STDERR_PRINTING_H */
