cmake_minimum_required(VERSION 3.16)
project(CoursOpenCv LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)

# Change this path to your actual OpenCV installation
set(OpenCV_DIR "C:/opencv/build")

find_package(OpenCV REQUIRED)

file(GLOB_RECURSE SRC CONFIGURE_DEPENDS src/*.cpp)
file(GLOB_RECURSE HEADERS CONFIGURE_DEPENDS include/*)

add_executable(CoursOpenCv ${HEADERS} ${SRC})

target_include_directories(CoursOpenCv PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${OpenCV_INCLUDE_DIRS}
)

target_link_libraries(CoursOpenCv PRIVATE
    ${OpenCV_LIBS}
)