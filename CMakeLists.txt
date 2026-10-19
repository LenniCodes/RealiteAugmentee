cmake_minimum_required(VERSION 3.10)
project(ReVA CXX)

set(CMAKE_CXX_STANDARD 17)            # std::filesystem (menu)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)       # optimisations par défaut : indispensable pour le temps réel
endif()

find_package(OpenCV REQUIRED)

# Programme principal
file(GLOB SRC CONFIGURE_DEPENDS src/*.cpp)
add_executable(CoursOpenCv ${SRC})
target_include_directories(CoursOpenCv PRIVATE include ${OpenCV_INCLUDE_DIRS})
target_link_libraries(CoursOpenCv PRIVATE ${OpenCV_LIBS})

# Outil : génère des vidéos de test de synthèse (pose connue, bruit, occultations)
add_executable(gen_synthetic tools/gen_synthetic.cpp src/pose.cpp)
target_include_directories(gen_synthetic PRIVATE include ${OpenCV_INCLUDE_DIRS})
target_link_libraries(gen_synthetic PRIVATE ${OpenCV_LIBS})
