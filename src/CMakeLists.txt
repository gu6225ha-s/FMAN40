set(SRC_FILES
    camera.h
    colmap.cpp
    colmap.h
    geometry.h
    image.h
    point3d.h
    polygon.h
    reconstruction.cpp
    reconstruction.h
    util.cpp
    util.h)

add_library(ppr
            ${SRC_FILES})
target_link_libraries(ppr Eigen3::Eigen)

add_executable(ppr-bin main.cpp)
target_link_libraries(ppr-bin ppr)
set_target_properties(ppr-bin PROPERTIES OUTPUT_NAME ppr)

add_custom_target(fmt
                  COMMAND clang-format
                  -i ${SRC_FILES} main.cpp
                  WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR})
