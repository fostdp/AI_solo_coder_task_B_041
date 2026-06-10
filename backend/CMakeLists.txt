cmake_minimum_required(VERSION 3.14)
project(TurbineMonitorMicroservices VERSION 1.0.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

message(STATUS "Build type: ${CMAKE_BUILD_TYPE}")
message(STATUS "System: ${CMAKE_SYSTEM_NAME}")

# ============================================================
# Dependencies
# ============================================================
find_package(Threads REQUIRED)

find_path(FFTW3_INCLUDE_DIR fftw3.h)
find_library(FFTW3_LIBRARY NAMES fftw3)

find_path(CLICKHOUSE_INCLUDE_DIR clickhouse/Client.h)
find_library(CLICKHOUSE_LIBRARY NAMES clickhouse-cpp)

find_path(JSON_INCLUDE_DIR nlohmann/json.hpp)

find_package(spdlog QUIET)
if(NOT spdlog_FOUND)
    find_path(SPDLOG_INCLUDE_DIR spdlog/spdlog.h)
    find_library(SPDLOG_LIBRARY NAMES spdlog)
    if(SPDLOG_INCLUDE_DIR AND SPDLOG_LIBRARY)
        message(STATUS "Found spdlog: ${SPDLOG_LIBRARY}")
    else()
        message(WARNING "spdlog not found - building without structured logging")
    endif()
endif()

find_path(PROMETHEUS_INCLUDE_DIR prometheus/registry.h)
find_library(PROMETHEUS_LIBRARY NAMES prometheus-cpp-pull prometheus-cpp-core)

include_directories(
    ${CMAKE_SOURCE_DIR}/common/include
    ${CMAKE_SOURCE_DIR}/include
    ${FFTW3_INCLUDE_DIR}
    ${CLICKHOUSE_INCLUDE_DIR}
    ${JSON_INCLUDE_DIR}
    ${SPDLOG_INCLUDE_DIR}
    ${PROMETHEUS_INCLUDE_DIR}
)

set(THIRD_PARTY_LIBS
    ${FFTW3_LIBRARY}
    ${CLICKHOUSE_LIBRARY}
    ${SPDLOG_LIBRARY}
    ${PROMETHEUS_LIBRARY}
    Threads::Threads
)

if(WIN32)
    list(APPEND THIRD_PARTY_LIBS ws2_32 mswsock)
endif()

# ============================================================
# Common static library (shared IPC + data structures)
# ============================================================
add_library(common STATIC
    common/src/ipc_queue.cpp
    src/data_structures.cpp
    src/config.cpp
    src/clickhouse_client.cpp
    src/logger.cpp
)

target_include_directories(common PUBLIC
    ${CMAKE_SOURCE_DIR}/common/include
    ${CMAKE_SOURCE_DIR}/include
)

# ============================================================
# Signal processing & algorithm library
# ============================================================
add_library(algo STATIC
    src/udp_server.cpp
    src/signal_processor.cpp
    src/feature_extractor.cpp
    src/cavitation_detector.cpp
    src/life_assessor.cpp
    src/alarm_manager.cpp
    src/api_server.cpp
    src/data_pipeline.cpp
)

target_include_directories(algo PUBLIC
    ${CMAKE_SOURCE_DIR}/include
)

# ============================================================
# Service 1: PXI Collector (高速采集 + 预处理)
# ============================================================
add_executable(pxi_collector
    services/pxi_collector/main.cpp
)
target_link_libraries(pxi_collector PRIVATE common algo ${THIRD_PARTY_LIBS})
set_target_properties(pxi_collector PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin/pxi_collector
)

# ============================================================
# Service 2: Cavitation Detector (空化识别 + 阶段判定)
# ============================================================
add_executable(cavitation_detector
    services/cavitation_detector/main.cpp
)
target_link_libraries(cavitation_detector PRIVATE common algo ${THIRD_PARTY_LIBS})
set_target_properties(cavitation_detector PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin/cavitation_detector
)

# ============================================================
# Service 3: Fatigue Evaluator (雨流计数 + 寿命评估)
# ============================================================
add_executable(fatigue_evaluator
    services/fatigue_evaluator/main.cpp
)
target_link_libraries(fatigue_evaluator PRIVATE common algo ${THIRD_PARTY_LIBS})
set_target_properties(fatigue_evaluator PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin/fatigue_evaluator
)

# ============================================================
# Service 4: Alarm Pusher (告警 + IEC 61850 推送)
# ============================================================
add_executable(alarm_pusher
    services/alarm_pusher/main.cpp
)
target_link_libraries(alarm_pusher PRIVATE common algo ${THIRD_PARTY_LIBS})
set_target_properties(alarm_pusher PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin/alarm_pusher
)

# ============================================================
# Service 5: API Gateway (REST聚合层)
# ============================================================
add_executable(api_gateway
    services/api_gateway/main.cpp
)
target_link_libraries(api_gateway PRIVATE common algo ${THIRD_PARTY_LIBS})
set_target_properties(api_gateway PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin/api_gateway
)

# ============================================================
# Simulator + Monolith (保留兼容)
# ============================================================
add_executable(turbine_monitor main.cpp)
target_link_libraries(turbine_monitor PRIVATE common algo ${THIRD_PARTY_LIBS})
set_target_properties(turbine_monitor PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin
)

add_executable(pxi_simulator simulators/pxi_simulator.cpp)
target_link_libraries(pxi_simulator PRIVATE common ${THIRD_PARTY_LIBS})
set_target_properties(pxi_simulator PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin
)

# ============================================================
# Service 6: Turbine Controller (MPC水轮机调节控制 - Feature 1)
# ============================================================
add_executable(turbine_controller
    services/turbine_controller/main.cpp
    src/turbine_mpc_controller.cpp
)
target_link_libraries(turbine_controller PRIVATE common algo ${THIRD_PARTY_LIBS})
set_target_properties(turbine_controller PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin/turbine_controller
)

# ============================================================
# Service 7: Robot Planner (水下机器人检修路径规划 - Feature 2)
# ============================================================
add_executable(robot_planner
    services/robot_planner/main.cpp
    src/robot_repair_planner.cpp
)
target_link_libraries(robot_planner PRIVATE common algo ${THIRD_PARTY_LIBS})
set_target_properties(robot_planner PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin/robot_planner
)

# ============================================================
# Service 8: Unit Scheduler (多机组MIP优化调度 - Feature 3)
# ============================================================
add_executable(unit_scheduler
    services/unit_scheduler/main.cpp
    src/unit_scheduler.cpp
)
target_link_libraries(unit_scheduler PRIVATE common algo ${THIRD_PARTY_LIBS})
set_target_properties(unit_scheduler PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin/unit_scheduler
)

# ============================================================
# Service 9: Acoustic Diagnosis (声纹特征库+度量学习诊断 - Feature 4)
# ============================================================
add_executable(acoustic_diagnosis
    services/acoustic_diagnosis/main.cpp
    src/acoustic_diagnosis.cpp
)
target_link_libraries(acoustic_diagnosis PRIVATE common algo ${THIRD_PARTY_LIBS})
set_target_properties(acoustic_diagnosis PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin/acoustic_diagnosis
)

# ============================================================
# Install / Package
# ============================================================
install(DIRECTORY ../config DESTINATION .)
install(DIRECTORY ../frontend DESTINATION .)
install(DIRECTORY ../tests DESTINATION . OPTIONAL)

# Print summary
message(STATUS "==================================================")
message(STATUS "  Turbine Monitor Microservices Build Targets:")
message(STATUS "    [1/11] pxi_collector         -> UDP采集+FFT/小波")
message(STATUS "    [2/11] cavitation_detector   -> AE/IF+工况归一化")
message(STATUS "    [3/11] fatigue_evaluator     -> 四点法雨流+Miner")
message(STATUS "    [4/11] alarm_pusher          -> 分级告警+IEC61850")
message(STATUS "    [5/11] api_gateway           -> REST API聚合")
message(STATUS "    [6/11] turbine_controller    -> MPC调节联动控制(F1)")
message(STATUS "    [7/11] robot_planner         -> 机器人检修路径规划(F2)")
message(STATUS "    [8/11] unit_scheduler        -> 多机组MIP优化调度(F3)")
message(STATUS "    [9/11] acoustic_diagnosis    -> 声纹度量学习诊断(F4)")
message(STATUS "    [10/11] turbine_monitor      -> 单体(兼容)")
message(STATUS "    [11/11] pxi_simulator        -> PXI流量模拟器")
message(STATUS "==================================================")
