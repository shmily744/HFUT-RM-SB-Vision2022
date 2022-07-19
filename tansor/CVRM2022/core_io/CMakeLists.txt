find_package(OpenCV 4 REQUIRED)

file(GLOB_RECURSE src *.cpp)

add_library(core_io OBJECT ${src})
target_link_libraries(core_io ${OpenCV_LIBS})
