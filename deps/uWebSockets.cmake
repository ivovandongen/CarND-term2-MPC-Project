include_guard_x()

# OpenSSL
find_package(OpenSSL 1.0.2 REQUIRED)

# ZLIB
find_package(ZLIB REQUIRED)

# LIBUV
find_package(LibUV REQUIRED)

set(UWEBSOCKETS_DIR ${CMAKE_SOURCE_DIR}/deps/uWebSockets)

add_library(libuWS STATIC
   ${UWEBSOCKETS_DIR}/src/Extensions.cpp
   ${UWEBSOCKETS_DIR}/src/Group.cpp
   ${UWEBSOCKETS_DIR}/src/WebSocketImpl.cpp
   ${UWEBSOCKETS_DIR}/src/Networking.cpp
   ${UWEBSOCKETS_DIR}/src/Hub.cpp
   ${UWEBSOCKETS_DIR}/src/Node.cpp
   ${UWEBSOCKETS_DIR}/src/WebSocket.cpp
   ${UWEBSOCKETS_DIR}/src/HTTPSocket.cpp
   ${UWEBSOCKETS_DIR}/src/Socket.cpp
   ${UWEBSOCKETS_DIR}/src/uUV.cpp
)

target_link_libraries(libuWS PUBLIC
    OpenSSL::Crypto
    OpenSSL::SSL
    ${ZLIB_LIBRARIES}
    ${LibUV_LIBRARY}
)

target_include_directories(libuWS
    PRIVATE ${OPENSSL_INCLUDE_DIR}
    PRIVATE ${ZLIB_INCLUDE_DIRS}
    PRIVATE ${LibUV_INCLUDE_DIR}
    SYSTEM PRIVATE ${UWEBSOCKETS_DIR}/src
)

target_include_directories(libuWS
    SYSTEM INTERFACE ${UWEBSOCKETS_DIR}/src
)