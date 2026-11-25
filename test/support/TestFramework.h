#pragma once

// Minimal xUnit-style framework so the suite builds with nothing but a C++
// compiler - no PlatformIO, no Unity, no installed packages. That matters here
// because the whole point of these tests is to be runnable in seconds on the
// development machine, including in CI.
//
// Usage:
//
//   TEST(Axis_Limits, clamps_above_maximum) {
//       Axis a(10, 170);
//       a.setPos(999);
//       CHECK_EQ(a.getPos(), 170);
//   }
//
// Each TEST registers itself; main() runs them all and reports a summary.
// A failing check records the failure and aborts that test only, so one broken
// expectation does not hide the rest of the suite.

#include <cmath>
#include <cstdio>
#include <functional>
#include <string>
#include <vector>

namespace testfw {

struct TestCase {
    std::string           suite;
    std::string           name;
    std::function<void()> fn;
};

// Function-local statics: one registry per program without any global object
// initialisation-order problems.
inline std::vector<TestCase>& registry() {
    static std::vector<TestCase> r;
    return r;
}

struct Failure {
    std::string message;
};

inline int& failureCount() {
    static int n = 0;
    return n;
}

inline int& checkCount() {
    static int n = 0;
    return n;
}

struct Registrar {
    Registrar(const char* suite, const char* name, std::function<void()> fn) {
        registry().push_back(TestCase{suite, name, std::move(fn)});
    }
};

}  // namespace testfw

// --- registration --------------------------------------------------------

#define TEST_CAT2(a, b) a##b
#define TEST_CAT(a, b) TEST_CAT2(a, b)

#define TEST(suite_name, test_name)                                        \
    static void TEST_CAT(test_fn_, __LINE__)();                            \
    static ::testfw::Registrar TEST_CAT(test_reg_, __LINE__)(               \
        #suite_name, #test_name, &TEST_CAT(test_fn_, __LINE__));            \
    static void TEST_CAT(test_fn_, __LINE__)()

// --- assertions ----------------------------------------------------------

#define FAIL_MSG(msg)                                                      \
    do {                                                                   \
        throw ::testfw::Failure{std::string(msg)};                          \
    } while (0)

#define CHECK(cond)                                                        \
    do {                                                                   \
        ::testfw::checkCount()++;                                          \
        if (!(cond)) {                                                     \
            FAIL_MSG(std::string("CHECK failed: ") + #cond +               \
                     " at " + __FILE__ + ":" + std::to_string(__LINE__));  \
        }                                                                  \
    } while (0)

#define CHECK_EQ(actual, expected)                                         \
    do {                                                                   \
        ::testfw::checkCount()++;                                          \
        const auto a_ = (actual);                                          \
        const auto e_ = (expected);                                        \
        if (!(a_ == e_)) {                                                 \
            FAIL_MSG(std::string("CHECK_EQ failed: ") + #actual +          \
                     " == " + #expected + " (got " +                       \
                     std::to_string(a_) + ", expected " +                  \
                     std::to_string(e_) + ") at " + __FILE__ + ":" +       \
                     std::to_string(__LINE__));                            \
        }                                                                  \
    } while (0)

#define CHECK_NE(actual, unwanted)                                         \
    do {                                                                   \
        ::testfw::checkCount()++;                                          \
        const auto a_ = (actual);                                          \
        const auto u_ = (unwanted);                                        \
        if (a_ == u_) {                                                    \
            FAIL_MSG(std::string("CHECK_NE failed: ") + #actual +          \
                     " != " + #unwanted + " (both " +                      \
                     std::to_string(a_) + ") at " + __FILE__ + ":" +       \
                     std::to_string(__LINE__));                            \
        }                                                                  \
    } while (0)

#define CHECK_GE(actual, bound)                                            \
    do {                                                                   \
        ::testfw::checkCount()++;                                          \
        const auto a_ = (actual);                                          \
        const auto b_ = (bound);                                           \
        if (!(a_ >= b_)) {                                                 \
            FAIL_MSG(std::string("CHECK_GE failed: ") + #actual +          \
                     " >= " + #bound + " (got " + std::to_string(a_) +     \
                     ") at " + __FILE__ + ":" +                            \
                     std::to_string(__LINE__));                            \
        }                                                                  \
    } while (0)

#define CHECK_LE(actual, bound)                                            \
    do {                                                                   \
        ::testfw::checkCount()++;                                          \
        const auto a_ = (actual);                                          \
        const auto b_ = (bound);                                           \
        if (!(a_ <= b_)) {                                                 \
            FAIL_MSG(std::string("CHECK_LE failed: ") + #actual +          \
                     " <= " + #bound + " (got " + std::to_string(a_) +     \
                     ") at " + __FILE__ + ":" +                            \
                     std::to_string(__LINE__));                            \
        }                                                                  \
    } while (0)

#define CHECK_NEAR(actual, expected, tol)                                  \
    do {                                                                   \
        ::testfw::checkCount()++;                                          \
        const double a_ = (double)(actual);                                \
        const double e_ = (double)(expected);                              \
        const double t_ = (double)(tol);                                   \
        if (std::fabs(a_ - e_) > t_) {                                     \
            FAIL_MSG(std::string("CHECK_NEAR failed: ") + #actual +        \
                     " ~= " + #expected + " (got " +                       \
                     std::to_string(a_) + ", expected " +                  \
                     std::to_string(e_) + ", tol " +                       \
                     std::to_string(t_) + ") at " + __FILE__ + ":" +       \
                     std::to_string(__LINE__));                            \
        }                                                                  \
    } while (0)

// --- runner --------------------------------------------------------------

inline int runAllTests() {
    int passed = 0;
    int failed = 0;
    std::string lastSuite;

    for (const auto& t : testfw::registry()) {
        if (t.suite != lastSuite) {
            std::printf("\n[%s]\n", t.suite.c_str());
            lastSuite = t.suite;
        }
        try {
            t.fn();
            ++passed;
            std::printf("  PASS  %s\n", t.name.c_str());
        } catch (const testfw::Failure& f) {
            ++failed;
            std::printf("  FAIL  %s\n        %s\n", t.name.c_str(),
                        f.message.c_str());
        } catch (const std::exception& e) {
            ++failed;
            std::printf("  FAIL  %s\n        unexpected exception: %s\n",
                        t.name.c_str(), e.what());
        } catch (...) {
            ++failed;
            std::printf("  FAIL  %s\n        unknown exception\n",
                        t.name.c_str());
        }
    }

    std::printf("\n--------------------------------------------------\n");
    std::printf("%d passed, %d failed, %d checks\n",
                passed, failed, testfw::checkCount());
    std::printf("--------------------------------------------------\n");
    return (failed == 0) ? 0 : 1;
}
