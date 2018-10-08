include_guard_x()

add_library(json INTERFACE)
target_include_directories(json SYSTEM INTERFACE ${CMAKE_SOURCE_DIR}/deps/json/src)
