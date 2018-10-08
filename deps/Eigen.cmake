include_guard_x()

add_library(Eigen INTERFACE)

target_include_directories(Eigen SYSTEM INTERFACE ${CMAKE_SOURCE_DIR}/deps/Eigen)

if (UNIX)
    # Silence warnings in Eigen
    add_compile_options(-Wno-deprecated-register)
endif()