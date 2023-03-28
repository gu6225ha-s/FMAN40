add_library(ppr
            camera.h
            colmap.cpp
            colmap.h
            str_util.cpp
            str_util.h)

add_executable(ppr-bin main.cpp)
target_link_libraries(ppr-bin ppr)
set_target_properties(ppr-bin PROPERTIES OUTPUT_NAME ppr)
