/*  trapfilter.h – C implementation of the fixed-rate trapezoidal filter
 *
 *  The original C++ file used templates to bake the three design
 *  parameters into the code.  In C we achieve the same effect with a
 *  pre-processor “instance generator”: define the three symbols before
 *  including this header and you will get a *specialised* set of
 *  static-inline functions with zero run-time overhead.
 *
 *      #define TRAP_FILTER_LENGTH  128   // L
 *      #define TRAP_FILTER_GAP      32   // G
 *      #define TRAP_FILTER_RATE      8   // R
 *      #include "trapfilter.h"
 *
 *  - L : integration length of each moving sum
 *  - G : gap between the two moving sums
 *  - R : decimation factor (number of successive samples that are first
 *        summed together to suppress quantisation noise)
 *
 *  The code below expects **12-bit raw ADC words** stored as uint16_t.
 *  If you keep the ESP-IDF driver in “TYPE2” (unit+data packed in
 *  `uint16_t`), you can call the filter directly on your `circ_buf[]`.
 *
 *  Written for ESP-IDF 5.4 (C17).
 *  Ji @ Duke University, 2025-06-25
 */
#ifndef TRAPFILTER_H
#define TRAPFILTER_H

#include <stdint.h>

/* --------------------------------------------------------------------
 * Compile-time parameter checks
 * ------------------------------------------------------------------ */
#ifndef TRAP_FILTER_LENGTH
#error "TRAP_FILTER_LENGTH not defined"
#endif
#ifndef TRAP_FILTER_GAP
#error "TRAP_FILTER_GAP not defined"
#endif
#ifndef TRAP_FILTER_RATE
#error "TRAP_FILTER_RATE not defined"
#endif

/* Derived constants – identical to the C++ version                   */
static const uint16_t _trap_len_shift = TRAP_FILTER_LENGTH - TRAP_FILTER_RATE + 1U;
static const uint16_t _trap_gap_shift = TRAP_FILTER_GAP - TRAP_FILTER_RATE + 1U;
static const uint16_t _trap_end_shift = 2U * TRAP_FILTER_LENGTH + TRAP_FILTER_GAP + 2U * TRAP_FILTER_RATE - 1U;

/* --------------------------------------------------------------------
 * Low-level moving-sum helper
 * ------------------------------------------------------------------ */
static inline int32_t trap_filter_sum_step(uint16_t **ppCursor)
{
    /* first sample (already pointed to)                                */
    int32_t sum = **ppCursor;

    /* include R-1 previous samples                                     */
#pragma GCC unroll 15 /* matches original pragma               */
    for (int i = 0; i < TRAP_FILTER_RATE - 1; ++i)
    {
        --(*ppCursor); /* move one sample back                  */
        sum += **ppCursor;
    }
    return sum;
}

/* --------------------------------------------------------------------
 * One trapezoidal-filter update (sample-by-sample)
 *
 *  *pp             – pointer that always tracks “current sample”
 *  *pAccu          – running filter output (int32_t to prevent overflow)
 *
 *  After the call,  *pp is advanced by the correct amount to be ready
 *  for the *next* new sample, so the caller only needs to set it once
 *  before the very first call.
 * ------------------------------------------------------------------ */
static inline void trap_filter_step(uint16_t **pp, int32_t *pAccu)
{
    /* +Σ entering leading window                                       */
    *pAccu += trap_filter_sum_step(pp);

    /* –Σ leaving leading window                                        */
    *pp -= _trap_len_shift;
    *pAccu -= trap_filter_sum_step(pp);

    /* –Σ entering trailing window                                      */
    *pp -= _trap_gap_shift;
    *pAccu -= trap_filter_sum_step(pp);

    /* +Σ leaving trailing window                                       */
    *pp -= _trap_len_shift;
    *pAccu += trap_filter_sum_step(pp);

    /* advance to next raw ADC sample                                   */
    *pp += _trap_end_shift;
}

#endif /* TRAPFILTER_H */
