add_subdirectory(src/BoW/DBoW2)
add_executable(test_bow src/BoW/test_bow.cpp src/BoW/ORBextractor.cpp)
target_link_libraries(test_bow ${catkin_LIBRARIES} ${OpenCV_LIBS} DBoW2)

add_subdirectory(src/GIST)

add_executable(test_delight src/DELIGHT/test_delight.cpp src/DELIGHT/DELIGHT.cpp )
target_link_libraries(test_delight ${catkin_LIBRARIES})

add_executable(test_m2dp src/M2DP/test_m2dp.cpp src/M2DP/M2DP.cpp)
target_link_libraries(test_m2dp ${catkin_LIBRARIES})

add_executable(test_sc src/SC/test_sc.cpp src/SC/SC.cpp)
target_link_libraries(test_sc ${catkin_LIBRARIES})

# add_executable(visualize_points src/utils/visualize_points.cpp)
# target_link_libraries(visualize_points ${catkin_LIBRARIES} ${OpenCV_LIBS})

