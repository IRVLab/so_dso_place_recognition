if (NOT DSO_PATH)
    set(DSO_PATH ~/Workspace/dso)
endif()
set(DSO_INCLUDE_DIRS "${DSO_PATH}/src" "${DSO_PATH}/thirdparty/Sophus")
set(DSO_LIBRARY_PATH "${DSO_PATH}/build/lib")
find_library(DSO_LIBRARY dso ${DSO_LIBRARY_PATH})
message("-- Found DSO library at \"${DSO_LIBRARY}\"")

find_package(Boost COMPONENTS system thread)
find_package(Pangolin 0.2 REQUIRED)

set(SODSOSystem_SOURCE_FILES
  src/SODSOSystem.cpp
  src/dso_helpers/SODSOSystemDebugStuff.cpp
  src/dso_helpers/SODSOSystemMarginalize.cpp
  src/dso_helpers/SODSOSystemOptimize.cpp
  src/dso_helpers/SODSOSystemOptPoint.cpp
)

include_directories(${DSO_INCLUDE_DIRS} src)
add_executable(so_dso_node src/so_dso_node.cpp ${SODSOSystem_SOURCE_FILES} src/output_wrappers/OutputWrapperLoop.cpp src/ScaleOptimizer.cpp)
target_link_libraries(so_dso_node
    ${DSO_LIBRARY}
    ${Pangolin_LIBRARIES}
    ${OpenCV_LIBS}
    ${PCL_LIBRARIES}
    ${catkin_LIBRARIES})
