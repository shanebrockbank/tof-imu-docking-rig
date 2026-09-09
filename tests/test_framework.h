#ifndef TESTS_TEST_FRAMEWORK_H
#define TESTS_TEST_FRAMEWORK_H

#include <stdio.h>
#include <math.h>

static int g_tests_run = 0;
static int g_tests_passed = 0;
static int g_current_test_failed = 0;

#define TEST(name) static void name(void)

#define RUN_TEST(name) do { \
    g_current_test_failed = 0; \
    g_tests_run++; \
    name(); \
    if (!g_current_test_failed) { \
        g_tests_passed++; \
    } else { \
        printf("FAIL: %s\n", #name); \
    } \
} while (0)

#define CHECK(cond) do { \
    if (!(cond)) { \
        g_current_test_failed = 1; \
        printf("  CHECK failed: %s (%s:%d)\n", #cond, __FILE__, __LINE__); \
    } \
} while (0)

#define CHECK_NEAR(a, b, tol) CHECK(fabs((a) - (b)) <= (tol))

#define TEST_SUMMARY() do { \
    printf("%d/%d tests passing\n", g_tests_passed, g_tests_run); \
    if (g_tests_passed != g_tests_run) return 1; \
} while (0)

#endif /* TESTS_TEST_FRAMEWORK_H */
