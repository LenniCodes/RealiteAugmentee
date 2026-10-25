cmake_minimum_required(VERSION 3.5)
project(CoursOpenCv)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set(SRC
	src/main.cpp
	src/findPaperSheet.cpp
	src/pose.cpp
	src/toGrayScale.cpp
	src/tracker.cpp
)

set(HEADERS
	include/utils.hpp
	include/pose.hpp
	include/tracker.hpp
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

