/*
 * Copyright (c) 2025 Ramon Cristopher Calam
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/ztest.h>
#include <zephyr/logging/log.h>
#include "fixed_math.h"

LOG_MODULE_REGISTER(test_fixed_math);

/**
 * @brief Test fixed-point conversion functions
 */
ZTEST(fixed_math_tests, test_conversions)
{
    // Test float to fixed conversion
    fixed_t fixed_val = fixed_from_float(1.5f);
    zassert_equal(fixed_val, FIXED_ONE + FIXED_HALF, "Float to fixed conversion failed");

    // Test fixed to float conversion
    float float_val = fixed_to_float(FIXED_ONE);
    zassert_within(float_val, 1.0f, 0.001f, "Fixed to float conversion failed");

    // Test integer conversions
    fixed_t from_int = fixed_from_int(42);
    zassert_equal(from_int, 42 << 16, "Integer to fixed conversion failed");

    int32_t to_int = fixed_to_int(FIXED_ONE + FIXED_HALF);
    zassert_equal(to_int, 1, "Fixed to integer truncation failed");

    int32_t to_int_round = fixed_to_int_round(FIXED_ONE + FIXED_HALF);
    zassert_equal(to_int_round, 2, "Fixed to integer rounding failed");
}

/**
 * @brief Test fixed-point arithmetic operations
 */
ZTEST(fixed_math_tests, test_arithmetic)
{
    fixed_t a = fixed_from_int(10);
    fixed_t b = fixed_from_int(3);

    // Test addition
    fixed_t sum = fixed_add(a, b);
    zassert_equal(fixed_to_int(sum), 13, "Fixed addition failed");

    // Test subtraction
    fixed_t diff = fixed_sub(a, b);
    zassert_equal(fixed_to_int(diff), 7, "Fixed subtraction failed");

    // Test multiplication
    fixed_t product = fixed_mul(a, b);
    zassert_equal(fixed_to_int(product), 30, "Fixed multiplication failed");

    // Test division
    fixed_t quotient = fixed_div(a, b);
    zassert_within(fixed_to_float(quotient), 3.333f, 0.01f, "Fixed division failed");
}

/**
 * @brief Test fixed-point comparison operations
 */
ZTEST(fixed_math_tests, test_comparisons)
{
    fixed_t a = fixed_from_int(5);
    fixed_t b = fixed_from_int(3);
    fixed_t c = fixed_from_int(5);

    // Test equality
    zassert_true(fixed_eq(a, c), "Fixed equality failed");
    zassert_false(fixed_eq(a, b), "Fixed inequality failed");

    // Test ordering
    zassert_true(fixed_lt(b, a), "Fixed less than failed");
    zassert_true(fixed_le(b, a), "Fixed less than or equal failed");
    zassert_true(fixed_le(a, c), "Fixed less than or equal (equal) failed");
    zassert_true(fixed_gt(a, b), "Fixed greater than failed");
    zassert_true(fixed_ge(a, b), "Fixed greater than or equal failed");
    zassert_true(fixed_ge(a, c), "Fixed greater than or equal (equal) failed");
}

/**
 * @brief Test fixed-point mathematical functions
 */
ZTEST(fixed_math_tests, test_math_functions)
{
    fixed_t val = fixed_from_int(9);

    // Test square root
    fixed_t sqrt_val = fixed_sqrt(val);
    zassert_within(fixed_to_float(sqrt_val), 3.0f, 0.1f, "Fixed square root failed");

    // Test absolute value
    fixed_t neg_val = fixed_from_int(-5);
    fixed_t abs_val = fixed_abs(neg_val);
    zassert_equal(fixed_to_int(abs_val), 5, "Fixed absolute value failed");
}

/**
 * @brief Test fixed-point statistical functions
 */
ZTEST(fixed_math_tests, test_statistics)
{
    fixed_t values[] = {fixed_from_int(1), fixed_from_int(2), fixed_from_int(3)};
    size_t count = 3;

    // Test mean calculation
    fixed_t mean = fixed_mean(values, count);
    zassert_within(fixed_to_float(mean), 2.0f, 0.01f, "Fixed mean calculation failed");

    // Test variance and standard deviation
    fixed_t variance = fixed_variance(values, count, mean);
    fixed_t stddev = fixed_stddev(values, count, mean);
    zassert_true(fixed_gt(variance, 0), "Fixed variance should be positive");
    zassert_true(fixed_gt(stddev, 0), "Fixed standard deviation should be positive");
}

/**
 * @brief Test edge cases and error conditions
 */
ZTEST(fixed_math_tests, test_edge_cases)
{
    // Test division by zero
    fixed_t result = fixed_div(FIXED_ONE, 0);
    zassert_equal(result, FIXED_MAX, "Division by zero should return max value");

    // Test integer division by zero
    result = fixed_div_int(FIXED_ONE, 0);
    zassert_equal(result, FIXED_MAX, "Integer division by zero should return max value");

    // Test square root of negative (should handle gracefully)
    fixed_t neg_sqrt = fixed_sqrt(fixed_from_int(-1));
    zassert_equal(neg_sqrt, 0, "Square root of negative should return 0");

    // Test overflow conditions (implementation-dependent)
    fixed_t large_val = FIXED_MAX;
    fixed_t product = fixed_mul(large_val, large_val);
    // Result should be clamped or wrapped, but not crash
    zassert_true(product != 0, "Multiplication overflow should be handled");
}

/* Test suite definition */
ZTEST_SUITE(fixed_math_tests, NULL, NULL, NULL, NULL, NULL);