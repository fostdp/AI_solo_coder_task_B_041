cmake_minimum_required(VERSION 3.10)
project(turbine_monitor CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

message(STATUS "Build type: ${CMAKE_BUILD_TYPE}")

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -Wpedantic")
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -O3 -march=native")
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -g -O0 -DDEBUG")

include_directories(${PROJECT_SOURCE_DIR}/include)

set(SOURCES
    src/main.cpp
    src/config.cpp
    src/udp_server.cpp
    src/clickhouse_client.cpp
    src/signal_processor.cpp
    src/cavitation_detector.cpp
    src/life_assessor.cpp
    src/alarm_manager.cpp
    src/api_server.cpp
    src/data_pipeline.cpp
)

set(HEADERS
    include/config.h
    include/data_structures.h
    include/udp_server.h
    include/clickhouse_client.h
    include/signal_processor.h
    include/cavitation_detector.h
    include/life_assessor.h
    include/alarm_manager.h
    include/api_server.h
    include/data_pipeline.h
)

add_executable(turbine_monitor ${SOURCES} ${HEADERS})

find_package(Threads REQUIRED)
target_link_libraries(turbine_monitor Threads::Threads)

find_package(PkgConfig QUIET)

find_library(FFTW3_LIBRARY NAMES fftw3 fftw3f)
find_path(FFTW3_INCLUDE_DIR NAMES fftw3.h)

if(FFTW3_LIBRARY AND FFTW3_INCLUDE_DIR)
    message(STATUS "Found FFTW3: ${FFTW3_LIBRARY}")
    include_directories(${FFTW3_INCLUDE_DIR})
    target_link_libraries(turbine_monitor ${FFTW3_LIBRARY})
    add_definitions(-DUSE_FFTW3)
else()
    message(WARNING "FFTW3 not found, using built-in FFT implementation")
endif()

find_library(CURL_LIBRARY NAMES curl)
find_path(CURL_INCLUDE_DIR NAMES curl/curl.h)

if(CURL_LIBRARY AND CURL_INCLUDE_DIR)
    message(STATUS "Found libcurl: ${CURL_LIBRARY}")
    include_directories(${CURL_INCLUDE_DIR})
    target_link_libraries(turbine_monitor ${CURL_LIBRARY})
    add_definitions(-DUSE_CURL)
else()
    message(WARNING "libcurl not found, ClickHouse client will be limited")
endif()

find_library(JSONCPP_LIBRARY NAMES jsoncpp)
find_path(JSONCPP_INCLUDE_DIR NAMES json/json.h)

if(JSONCPP_LIBRARY AND JSONCPP_INCLUDE_DIR)
    message(STATUS "Found jsoncpp: ${JSONCPP_LIBRARY}")
    include_directories(${JSONCPP_INCLUDE_DIR})
    target_link_libraries(turbine_monitor ${JSONCPP_LIBRARY})
    add_definitions(-DUSE_JSONCPP)
else()
    message(WARNING "jsoncpp not found, using built-in JSON parser")
endif()

if(WIN32)
    target_link_libraries(turbine_monitor ws2_32 mswsock)
    add_definitions(-DWIN32_LEAN_AND_MEAN -D_CRT_SECURE_NO_WARNINGS)
endif()

if(UNIX AND NOT APPLE)
    target_link_libraries(turbine_monitor rt)
endif()

target_compile_definitions(turbine_monitor PRIVATE
    PROJECT_VERSION="1.0.0"
    PROJECT_NAME="turbine_monitor"
)

install(TARGETS turbine_monitor
    RUNTIME DESTINATION bin
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
)

install(DIRECTORY ${PROJECT_SOURCE_DIR}/../config/
    DESTINATION etc/turbine_monitor
    FILES_MATCHING PATTERN "*.json"
)

enable_testing()

message(STATUS "========================================")
message(STATUS "Turbine Monitor Build Configuration")
message(STATUS "========================================")
message(STATUS "C++ Compiler: ${CMAKE_CXX_COMPILER_ID} ${CMAKE_CXX_COMPILER_VERSION}")
message(STATUS "Build Type: ${CMAKE_BUILD_TYPE}")
message(STATUS "FFTW3: ${FFTW3_LIBRARY}")
message(STATUS "libcurl: ${CURL_LIBRARY}")
message(STATUS "jsoncpp: ${JSONCPP_LIBRARY}")
message(STATUS "========================================")
