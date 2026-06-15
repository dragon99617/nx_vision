cmake_minimum_required(VERSION 3.16)

project(nx_vision VERSION 0.1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(OpenCV REQUIRED COMPONENTS core imgproc highgui calib3d videoio imgcodecs)
find_package(Threads REQUIRED)

option(NXVISION_WITH_ORBBEC "Build native Orbbec SDK camera backend" ON)
set(ORBBECSDK_ROOT "" CACHE PATH "Path to an Orbbec SDK v2 install or extracted package")
if(NXVISION_WITH_ORBBEC)
    if(ORBBECSDK_ROOT)
        list(PREPEND CMAKE_PREFIX_PATH "${ORBBECSDK_ROOT}" "${ORBBECSDK_ROOT}/lib")
    elseif(DEFINED ENV{ORBBECSDK_ROOT})
        list(PREPEND CMAKE_PREFIX_PATH "$ENV{ORBBECSDK_ROOT}" "$ENV{ORBBECSDK_ROOT}/lib")
    endif()

    find_package(OrbbecSDK CONFIG QUIET)
    if(OrbbecSDK_FOUND)
        message(STATUS "Orbbec SDK found; native camera backend enabled")
    else()
        message(WARNING "Orbbec SDK not found; native camera backend disabled")
        set(NXVISION_WITH_ORBBEC OFF)
    endif()
endif()

add_library(nx_vision_core
    src/camera/orbbec_camera.cpp
    src/comm/gimbal_protocol.cpp
    src/comm/serial_port.cpp
    src/common/app_context.cpp
    src/common/config.cpp
    src/debug_view/overlay_drawer.cpp
    src/debug_view/panel_layout.cpp
    src/debug_view/visual_debugger.cpp
    src/pipeline/aim_pipeline.cpp
    src/pose/angle_projector.cpp
    src/pose/laser_compensator.cpp
    src/pose/pnp_solver.cpp
    src/tasks/adv_circle_sync.cpp
    src/tasks/adv_one_lap.cpp
    src/tasks/adv_two_laps.cpp
    src/tasks/basic_static_aim.cpp
    src/tasks/basic_trace_only.cpp
    src/tasks/basic_track_aim.cpp
    src/tasks/task_base.cpp
    src/vision/laser_detector.cpp
    src/vision/preprocess.cpp
    src/vision/rectangle_detector.cpp
    src/vision/target_tracker.cpp
)

target_include_directories(nx_vision_core PUBLIC include)
target_link_libraries(nx_vision_core PUBLIC ${OpenCV_LIBS} Threads::Threads)
if(NXVISION_WITH_ORBBEC)
    target_compile_definitions(nx_vision_core PUBLIC NXVISION_WITH_ORBBEC=1)
    target_link_libraries(nx_vision_core PUBLIC ob::OrbbecSDK)
endif()

function(add_nx_app target source)
    add_executable(${target} ${source})
    target_link_libraries(${target} PRIVATE nx_vision_core)
endfunction()

add_nx_app(nx_runtime apps/runtime/main.cpp)
add_nx_app(nx_debug_studio apps/debug_studio/main.cpp)
add_nx_app(nx_replay_viewer apps/replay_viewer/main.cpp)
add_nx_app(nx_calibration apps/calibration_studio/main.cpp)
add_nx_app(nx_task_runner apps/task_runner/main.cpp)
add_nx_app(nx_camera_smoke apps/camera_smoke/main.cpp)

option(NXVISION_BUILD_TESTS "Build lightweight unit tests" ON)
if(NXVISION_BUILD_TESTS)
    enable_testing()
    add_executable(test_gimbal_protocol tests/test_gimbal_protocol.cpp)
    target_link_libraries(test_gimbal_protocol PRIVATE nx_vision_core)
    add_test(NAME test_gimbal_protocol COMMAND test_gimbal_protocol)
endif()
