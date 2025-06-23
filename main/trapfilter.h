/*
 * Trapezoidal Filter for ADC Data Processing
 *
 * The ESP32 stores ADC data in continuous mode. This filter implements
 * a trapezoidal (moving average) filter for noise reduction and signal processing.
 *
 * Converted from C++ templates to C macros and functions for compatibility
 * with the ESP-IDF C-based ADC continuous reading application.
 */

#ifndef TRAPFILTER_H
#define TRAPFILTER_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

// Default filter parameters - can be overridden before including this header
#ifndef TRAP_FILTER_LENGTH
#define TRAP_FILTER_LENGTH 64
#endif

#ifndef TRAP_FILTER_GAP
#define TRAP_FILTER_GAP 32
#endif

#ifndef TRAP_FILTER_RATE
#define TRAP_FILTER_RATE 8
#endif

// Trapezoidal filter state structure
typedef struct
{
    int32_t filter_value;     // Current filter output value
    size_t buffer_pos;        // Current position in circular buffer
    size_t samples_processed; // Total samples processed
    bool initialized;         // Filter initialization flag
} trap_filter_t;

// Filter sum step function - C version of template
// Returns sum over FILTER_RATE samples, going backwards from current position
static inline int32_t filter_sum_step(uint16_t *buffer, size_t buffer_size, size_t *pos, int filter_rate)
{
    int32_t val_sum = buffer[*pos];

    for (int i = 0; i < filter_rate - 1; ++i)
    {
        // Move backwards in circular buffer
        *pos = (*pos == 0) ? (buffer_size - 1) : (*pos - 1);
        val_sum += buffer[*pos];
    }

    return val_sum;
}

// Initialize trapezoidal filter
static inline void trap_filter_init(trap_filter_t *filter)
{
    filter->filter_value = 0;
    filter->buffer_pos = 0;
    filter->samples_processed = 0;
    filter->initialized = false;
}

// Apply trapezoidal filter step
// This implements the classic trapezoidal filter algorithm:
// Output = sum(recent samples) - sum(older samples) - sum(gap samples) + sum(oldest samples)
static inline int32_t trap_filter_step(trap_filter_t *filter, uint16_t *circ_buffer, size_t buffer_size, size_t current_write_pos)
{
    const int filter_length_shift = TRAP_FILTER_LENGTH - TRAP_FILTER_RATE + 1;
    const int filter_gap_shift = TRAP_FILTER_GAP - TRAP_FILTER_RATE + 1;
    const int filter_end_shift = 2 * TRAP_FILTER_LENGTH + TRAP_FILTER_GAP + 2 * TRAP_FILTER_RATE - 1;

    // Initialize filter position on first call
    if (!filter->initialized)
    {
        filter->buffer_pos = current_write_pos;
        filter->initialized = true;

        // Pre-fill the filter with initial values to avoid startup transients
        size_t temp_pos = current_write_pos;
        filter->filter_value = filter_sum_step(circ_buffer, buffer_size, &temp_pos, TRAP_FILTER_RATE);
        return filter->filter_value;
    }

    // Update buffer position to latest write position
    filter->buffer_pos = current_write_pos;
    size_t work_pos = filter->buffer_pos;

    // Step 1: Add samples entering filter window (most recent)
    filter->filter_value += filter_sum_step(circ_buffer, buffer_size, &work_pos, TRAP_FILTER_RATE);

    // Step 2: Subtract samples leaving first filter section
    for (int i = 0; i < filter_length_shift; i++)
    {
        work_pos = (work_pos == 0) ? (buffer_size - 1) : (work_pos - 1);
    }
    filter->filter_value -= filter_sum_step(circ_buffer, buffer_size, &work_pos, TRAP_FILTER_RATE);

    // Step 3: Subtract samples entering gap section
    for (int i = 0; i < filter_gap_shift; i++)
    {
        work_pos = (work_pos == 0) ? (buffer_size - 1) : (work_pos - 1);
    }
    filter->filter_value -= filter_sum_step(circ_buffer, buffer_size, &work_pos, TRAP_FILTER_RATE);

    // Step 4: Add samples leaving second filter section (oldest)
    for (int i = 0; i < filter_length_shift; i++)
    {
        work_pos = (work_pos == 0) ? (buffer_size - 1) : (work_pos - 1);
    }
    filter->filter_value += filter_sum_step(circ_buffer, buffer_size, &work_pos, TRAP_FILTER_RATE);

    filter->samples_processed++;
    return filter->filter_value;
}

// Get normalized filter output (divide by effective filter length)
static inline int32_t trap_filter_get_normalized(trap_filter_t *filter)
{
    // Effective filter length is 2 * FILTER_LENGTH * FILTER_RATE
    int32_t effective_length = 2 * TRAP_FILTER_LENGTH * TRAP_FILTER_RATE;
    return filter->filter_value / effective_length;
}

#endif // TRAPFILTER_H
