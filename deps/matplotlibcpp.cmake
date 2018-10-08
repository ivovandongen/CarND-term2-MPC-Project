include_guard_x()

# We need the python header
find_package(PythonLibs 2.7 REQUIRED)

add_library(matplotlibcpp INTERFACE)

target_include_directories(matplotlibcpp SYSTEM INTERFACE
        ${CMAKE_SOURCE_DIR}/deps/matplotlibcpp
        ${PYTHON_INCLUDE_DIRS}
        )

target_link_libraries(matplotlibcpp INTERFACE ${PYTHON_LIBRARIES})

# No numpy
target_compile_definitions(matplotlibcpp INTERFACE -DWITHOUT_NUMPY=1)
