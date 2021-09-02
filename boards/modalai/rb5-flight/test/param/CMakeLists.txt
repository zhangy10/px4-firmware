
project( param-test )

add_definitions(-D__EXPORT="")

set(CMAKE_CXX_STANDARD 11)

add_executable(param-test tinybson.cpp param-test.cpp)
