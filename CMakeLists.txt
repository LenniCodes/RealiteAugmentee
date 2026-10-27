cmake_minimum_required(VERSION 3.5)
project(CoursOpenCv)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set(SRC
    src/app/main.cpp
    src/core/quad_utils.cpp
    src/io/camera_loader.cpp
    src/io/frame_source.cpp
    src/geometry/pose_estimator.cpp
    src/vision/edge_refiner.cpp
    src/vision/sheet_detector.cpp
    src/tracking/one_euro_filter.cpp
    src/tracking/sheet_tracker.cpp
    src/render/overlay.cpp
)

set(HEADERS
    include/core/types.hpp
    include/core/quad_utils.hpp
    include/io/frame_source.hpp
    include/io/camera_loader.hpp
    include/geometry/pose_estimator.hpp
    include/vision/edge_refiner.hpp
    include/vision/sheet_detector.hpp
    include/tracking/one_euro_filter.hpp
    include/tracking/sheet_tracker.hpp
    include/render/overlay.hpp
)

add_executable(CoursOpenCv ${HEADERS} ${SRC})

target_include_directories(CoursOpenCv PUBLIC include)

find_package(OpenCV QUIET)

if(OpenCV_FOUND)
    target_include_directories(CoursOpenCv PUBLIC ${OpenCV_INCLUDE_DIRS})
    target_link_libraries(CoursOpenCv ${OpenCV_LIBS})
else()
    find_package(PkgConfig REQUIRED)
    pkg_check_modules(OPENCV4 REQUIRED opencv4)
    target_include_directories(CoursOpenCv PUBLIC ${OPENCV4_INCLUDE_DIRS})
    target_link_directories(CoursOpenCv PUBLIC ${OPENCV4_LIBRARY_DIRS})
    target_link_libraries(CoursOpenCv ${OPENCV4_LIBRARIES})
endif()

