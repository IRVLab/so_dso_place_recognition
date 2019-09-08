list(APPEND CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/loop_closure/src/GIST/cmake")

if(MSVC)
  set(CMAKE_CXX_FLAGS "/EHsc")
else()
  set(CMAKE_CXX_FLAGS "-std=c++11 -fno-operator-names")
endif()

find_package(FFTW REQUIRED)
include_directories(${FFTW_INCLUDES})

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_BINARY_DIR})

set(SRC_LIST
  src/gist.cpp
  src/libgist.cpp
  src/standalone_image.cpp
  src/test_gist.cpp
)
set(HEADER_LIST
  include/gist.h
  include/libgist.h
  include/standalone_image.h
)

add_executable(test_gist ${HEADER_LIST} ${SRC_LIST})
target_link_libraries(test_gist ${catkin_LIBRARIES} ${FFTWF_LIB} ${OpenCV_LIBS})
