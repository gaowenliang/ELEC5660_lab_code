add_library(${PROJECT_NAME}
    bluefox2.cpp
    bluefox2_ros.cpp
    bluefox2_setting.cpp
    single/single_node.cpp
#    stereo/stereo_node.cpp
#    single/single_nodelet.cpp
#    stereo/stereo_nodelet.cpp
#    multi/multi_node.cpp
    )
target_link_libraries(${PROJECT_NAME}
    ${mvIMPACT_LIBRARIES}
    ${catkin_LIBRARIES}
    )

# single node
add_executable(single_node single/single_main.cpp)
target_link_libraries(single_node ${PROJECT_NAME})

# stereo node
#add_executable(stereo_node stereo/stereo_main.cpp)
#target_link_libraries(stereo_node ${PROJECT_NAME})

# multi node
#add_executable(multi_node multi/multi_main.cpp)
#target_link_libraries(multi_node ${PROJECT_NAME})

# list_mvdevice
add_executable(list_cameras list_cameras.cpp)
target_link_libraries(list_cameras ${PROJECT_NAME} ${Boost_LIBRARIES})
