cmake_minimum_required(VERSION 3.5)
project(CoursOpenCv)

file(GLOB_RECURSE SRC src/*)
file(GLOB_RECURSE HEADERS include/*)

add_executable(CoursOpenCv ${LIB_HEADERS} ${HEADERS} ${SRC})

find_package( OpenCV REQUIRED )
include_directories( ${OpenCV_INCLUDE_DIRS} )

target_link_directories(CoursOpenCv PUBLIC .)
target_link_libraries(CoursOpenCv ${OpenCV_LIBS})
target_include_directories(CoursOpenCv PUBLIC include ${OpenCV_INCLUDE_DIRS})
