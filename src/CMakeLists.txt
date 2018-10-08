# Submodules
git_include(Eigen)

find_package(IPOPT 3.12 REQUIRED)

add_library(core STATIC
    ${CMAKE_SOURCE_DIR}/src/polyline.hpp
    ${CMAKE_SOURCE_DIR}/src/MPC.hpp
    ${CMAKE_SOURCE_DIR}/src/MPC.cpp
    )

target_include_directories(core
    PUBLIC ${CMAKE_SOURCE_DIR}/src
    )

target_link_libraries(core PUBLIC
    Eigen
    json
    ipopt
    )

if (DO_CLANG_TIDY)
    set_target_properties(core PROPERTIES CXX_CLANG_TIDY "${DO_CLANG_TIDY}")
endif ()