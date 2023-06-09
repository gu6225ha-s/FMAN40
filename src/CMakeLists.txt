set(SRC_FILES
    colmap.cpp
    colmap.h
    geometry.h
    image.cpp
    image.h
    mesh.h
    mesh.cpp
    polygon.h
    reconstruction.cpp
    reconstruction.h
    util.cpp
    util.h)

add_library(ppr
            ${SRC_FILES})
target_link_libraries(ppr Eigen3::Eigen JPEG::JPEG)
target_compile_options(ppr PRIVATE -Wall)

add_executable(ppr-bin main.cpp)
target_link_libraries(ppr-bin ppr)
set_target_properties(ppr-bin PROPERTIES OUTPUT_NAME ppr)

add_custom_target(fmt
                  COMMAND clang-format
                  -i ${SRC_FILES} main.cpp
                  WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR})

find_package(Doxygen)
if (DOXYGEN_FOUND)
  add_custom_target(doc
                    COMMAND echo "INPUT=${CMAKE_CURRENT_SOURCE_DIR}" | ${DOXYGEN_EXECUTABLE} -
                    WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR})
endif (DOXYGEN_FOUND)
