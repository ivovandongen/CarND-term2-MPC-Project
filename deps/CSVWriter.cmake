include_guard_x()

add_library(CSVWriter INTERFACE)
target_include_directories(CSVWriter SYSTEM INTERFACE ${CMAKE_SOURCE_DIR}/deps/CSVWriter/include)
