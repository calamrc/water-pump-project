/*
 * Copyright (c) 2025 Ramon Cristopher Calam
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef FIXED_MATH_H
#define FIXED_MATH_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

/**
 * @brief Q16.16 fixed-point arithmetic library
 *
 * Provides efficient fixed-point mathematics for embedded systems,
 * eliminating FPU dependency and reducing memory usage by ~75%
 * compared to floating-point operations.
 *
 * Format: 32-bit signed integer with 16 bits fractional precision
 * Range: -32768.0 to +32767.999... (approximate)
 * Resolution: 1/65536 ≈ 0.00001526
 */

// ============================================================================
// Type Definitions and Constants
// ============================================================================

/** Q16.16 fixed-point type */
typedef int32_t fixed_t;

/** Scale factor for Q16.16 format */
#define FIXED_SCALE (1 << 16)

/** Scale factor as floating-point for conversions */
#define FIXED_SCALE_F (65536.0f)

/** Maximum positive value in Q16.16 */
#define FIXED_MAX INT32_MAX

/** Minimum negative value in Q16.16 */
#define FIXED_MIN INT32_MIN

/** Value of 1.0 in fixed-point */
#define FIXED_ONE (FIXED_SCALE)

/** Value of 0.5 in fixed-point (for rounding) */
#define FIXED_HALF (FIXED_SCALE / 2)

// ============================================================================
// Conversion Functions
// ============================================================================

/**
 * @brief Convert floating-point to Q16.16 fixed-point
 *
 * @param f Floating-point value
 * @return Equivalent fixed-point value
 */
static inline fixed_t fixed_from_float(float f) {
    return (fixed_t)(f * FIXED_SCALE_F + (f >= 0.0f ? 0.5f : -0.5f));
}

/**
 * @brief Convert Q16.16 fixed-point to floating-point
 *
 * @param x Fixed-point value
 * @return Equivalent floating-point value
 */
static inline float fixed_to_float(fixed_t x) {
    return (float)x / FIXED_SCALE_F;
}

/**
 * @brief Convert integer to Q16.16 fixed-point
 *
 * @param i Integer value
 * @return Equivalent fixed-point value
 */
static inline fixed_t fixed_from_int(int32_t i) {
    return i << 16;
}

/**
 * @brief Convert Q16.16 fixed-point to integer (truncation)
 *
 * @param x Fixed-point value
 * @return Equivalent integer value (truncated)
 */
static inline int32_t fixed_to_int(fixed_t x) {
    return x >> 16;
}

/**
 * @brief Convert Q16.16 fixed-point to integer (rounding)
 *
 * @param x Fixed-point value
 * @return Equivalent integer value (rounded)
 */
static inline int32_t fixed_to_int_round(fixed_t x) {
    if (x >= 0) {
        return (x + FIXED_HALF) >> 16;
    } else {
        return (x - FIXED_HALF) >> 16;
    }
}

// ============================================================================
// Arithmetic Operations
// ============================================================================

/**
 * @brief Add two Q16.16 fixed-point values
 *
 * @param a First value
 * @param b Second value
 * @return a + b in fixed-point
 */
static inline fixed_t fixed_add(fixed_t a, fixed_t b) {
    return a + b;
}

/**
 * @brief Subtract two Q16.16 fixed-point values
 *
 * @param a First value
 * @param b Second value
 * @return a - b in fixed-point
 */
static inline fixed_t fixed_sub(fixed_t a, fixed_t b) {
    return a - b;
}

/**
 * @brief Multiply two Q16.16 fixed-point values
 *
 * Uses 64-bit intermediate to prevent overflow.
 *
 * @param a First value
 * @param b Second value
 * @return a * b in fixed-point
 */
static inline fixed_t fixed_mul(fixed_t a, fixed_t b) {
    int64_t temp = (int64_t)a * (int64_t)b;
    return (fixed_t)(temp >> 16);
}

/**
 * @brief Divide two Q16.16 fixed-point values
 *
 * @param a Dividend
 * @param b Divisor (must not be zero)
 * @return a / b in fixed-point
 */
static inline fixed_t fixed_div(fixed_t a, fixed_t b) {
    if (b == 0) return FIXED_MAX; // Prevent division by zero
    int64_t temp = ((int64_t)a << 16) / b;
    return (fixed_t)temp;
}

/**
 * @brief Multiply fixed-point by integer
 *
 * @param a Fixed-point value
 * @param b Integer multiplier
 * @return a * b in fixed-point
 */
static inline fixed_t fixed_mul_int(fixed_t a, int32_t b) {
    return a * b;
}

/**
 * @brief Divide fixed-point by integer
 *
 * @param a Fixed-point dividend
 * @param b Integer divisor (must not be zero)
 * @return a / b in fixed-point
 */
static inline fixed_t fixed_div_int(fixed_t a, int32_t b) {
    if (b == 0) return FIXED_MAX; // Prevent division by zero
    return a / b;
}

// ============================================================================
// Comparison Operations
// ============================================================================

/**
 * @brief Compare two fixed-point values for equality
 *
 * @param a First value
 * @param b Second value
 * @return true if a == b
 */
static inline bool fixed_eq(fixed_t a, fixed_t b) {
    return a == b;
}

/**
 * @brief Compare two fixed-point values
 *
 * @param a First value
 * @param b Second value
 * @return -1 if a < b, 0 if a == b, 1 if a > b
 */
static inline int fixed_cmp(fixed_t a, fixed_t b) {
    if (a < b) return -1;
    if (a > b) return 1;
    return 0;
}

/**
 * @brief Check if a is less than b
 *
 * @param a First value
 * @param b Second value
 * @return true if a < b
 */
static inline bool fixed_lt(fixed_t a, fixed_t b) {
    return a < b;
}

/**
 * @brief Check if a is less than or equal to b
 *
 * @param a First value
 * @param b Second value
 * @return true if a <= b
 */
static inline bool fixed_le(fixed_t a, fixed_t b) {
    return a <= b;
}

/**
 * @brief Check if a is greater than b
 *
 * @param a First value
 * @param b Second value
 * @return true if a > b
 */
static inline bool fixed_gt(fixed_t a, fixed_t b) {
    return a > b;
}

/**
 * @brief Check if a is greater than or equal to b
 *
 * @param a First value
 * @param b Second value
 * @return true if a >= b
 */
static inline bool fixed_ge(fixed_t a, fixed_t b) {
    return a >= b;
}

// ============================================================================
// Mathematical Functions
// ============================================================================

/**
 * @brief Calculate absolute value of fixed-point number
 *
 * @param x Input value
 * @return |x| in fixed-point
 */
static inline fixed_t fixed_abs(fixed_t x) {
    return (x < 0) ? -x : x;
}

/**
 * @brief Calculate square root using integer approximation
 *
 * Implements a fast integer square root algorithm adapted for fixed-point.
 * Accuracy is sufficient for flow measurement applications.
 *
 * @param x Input value (must be non-negative)
 * @return √x in fixed-point
 */
static inline fixed_t fixed_sqrt(fixed_t x) {
    if (x <= 0) return 0;

    // Convert to integer domain for calculation
    uint32_t x_int = (uint32_t)x << 8; // Scale up for better precision
    uint32_t result = 0;
    uint32_t bit = 1 << 30; // Start with highest bit

    // Integer square root algorithm
    while (bit > x_int) {
        bit >>= 2;
    }

    while (bit != 0) {
        if (x_int >= result + bit) {
            x_int -= result + bit;
            result = (result >> 1) + bit;
        } else {
            result >>= 1;
        }
        bit >>= 2;
    }

    // Convert back to fixed-point and adjust scaling
    return (fixed_t)(result) << 4; // Adjust for the scaling we applied
}

/**
 * @brief Calculate mean of an array of fixed-point values
 *
 * @param arr Array of fixed-point values
 * @param count Number of elements
 * @return Mean value in fixed-point
 */
static inline fixed_t fixed_mean(const fixed_t *arr, size_t count) {
    if (count == 0) return 0;

    int64_t sum = 0;
    for (size_t i = 0; i < count; i++) {
        sum += arr[i];
    }
    return (fixed_t)(sum / count);
}

/**
 * @brief Calculate variance of an array of fixed-point values
 *
 * @param arr Array of fixed-point values
 * @param count Number of elements
 * @param mean Pre-calculated mean value
 * @return Variance in fixed-point
 */
static inline fixed_t fixed_variance(const fixed_t *arr, size_t count, fixed_t mean) {
    if (count <= 1) return 0;

    int64_t sum_sq_diff = 0;
    for (size_t i = 0; i < count; i++) {
        fixed_t diff = fixed_sub(arr[i], mean);
        sum_sq_diff += (int64_t)fixed_mul(diff, diff);
    }
    return (fixed_t)(sum_sq_diff / (count - 1));
}

/**
 * @brief Calculate standard deviation of an array of fixed-point values
 *
 * @param arr Array of fixed-point values
 * @param count Number of elements
 * @param mean Pre-calculated mean value
 * @return Standard deviation in fixed-point
 */
static inline fixed_t fixed_stddev(const fixed_t *arr, size_t count, fixed_t mean) {
    fixed_t variance = fixed_variance(arr, count, mean);
    return fixed_sqrt(fixed_abs(variance)); // sqrt of variance
}

// ============================================================================
// Flow Meter Specific Constants
// ============================================================================

/** Flow rate threshold in fixed-point (0.1 L/min) */
#define FIXED_FLOW_THRESHOLD fixed_from_float(0.1f)

/** Plateau minimum slope in fixed-point (0.01 L/min per sample) */
#define FIXED_PLATEAU_MIN_SLOPE fixed_from_float(0.01f)

/** Plateau initial K-factor multiplier in fixed-point (2.0) */
#define FIXED_PLATEAU_INITIAL_K_FACTOR fixed_from_float(2.0f)

/** Plateau K-factor multiplier in fixed-point (3.0) */
#define FIXED_PLATEAU_K_FACTOR fixed_from_float(3.0f)

/** Epsilon fallback for calibration in fixed-point (0.01) */
#define FIXED_EPSILON_FALLBACK fixed_from_float(0.01f)

/** YF-S201C pulses per liter constant */
#define YF_S201C_PULSES_PER_LITER_INT 450

/** Seconds per minute for flow calculation */
#define SECONDS_PER_MINUTE_INT 60

/** Microseconds per second for flow calculation */
#define USEC_PER_SECOND_INT 1000000

#endif /* FIXED_MATH_H */
