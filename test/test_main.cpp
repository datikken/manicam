// Entry point for the host-side test suite.
//
// Every test file registers its cases with the framework; this just runs them
// and turns the result into a process exit code, so a CI step can fail the
// build when a test breaks.

#include "TestFramework.h"

int main() {
    return runAllTests();
}
