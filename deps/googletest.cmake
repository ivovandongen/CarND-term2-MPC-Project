include_guard_x()

set(GTEST_DIR ${CMAKE_SOURCE_DIR}/deps/googletest/googletest)

add_library(gtest STATIC
    ${GTEST_DIR}/src/gtest.cc
    ${GTEST_DIR}/src/gtest-death-test.cc
    ${GTEST_DIR}/src/gtest-filepath.cc
    ${GTEST_DIR}/src/gtest-port.cc
    ${GTEST_DIR}/src/gtest-printers.cc
    ${GTEST_DIR}/src/gtest-test-part.cc
    ${GTEST_DIR}/src/gtest-typed-test.cc
)

target_include_directories(gtest
    PRIVATE ${GTEST_DIR}/include
    PRIVATE ${GTEST_DIR}
)

target_include_directories(gtest
    SYSTEM INTERFACE ${GTEST_DIR}/include
)