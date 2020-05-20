

project(mapping)

### TODO: convert the gtsam::pose3 to tf
### TODO: include cam model and SR_io 

################## compile #########################
# add_executable(mapping_PLY mapping_PLY.cpp)
# target_link_libraries(mapping_PLY gtsam vro sr4k_io ${OpenCV_LIBS})

# add_executable(mapping_PCD mapping_PCD.cpp)
# target_link_libraries(mapping_PCD gtsam_graph vro sr4k_io)

# add_executable(mapping_PLY_rs mapping_PLY_rs.cpp)
# target_link_libraries(mapping_PLY_rs gtsam vro sr4k_io ${OpenCV_LIBS})

# add_executable(mapping_PCD_rs mapping_PCD_rs.cpp)
# target_link_libraries(mapping_PCD_rs gtsam_graph vro sr4k_io ${OpenCV_LIBS})

add_executable(pcd2mesh pcd2mesh.cpp)
target_link_libraries(pcd2mesh ${PCL_LIBRARIES})

add_executable(pub_pcd pub_pcd.cpp)
target_link_libraries(pub_pcd ${catkin_LIBRARIES} ${PCL_LIBRARIES})

add_executable(pcd_filter pcd_filter.cpp)
target_link_libraries(pcd_filter ${PCL_LIBRARIES})

add_executable(mapping_bag_ply mapping_bag_ply.cpp)
target_link_libraries(mapping_bag_ply ${catkin_LIBRARIES} ${OpenCV_LIBS})

add_executable(traj_2_ply traj_2_ply.cpp)
target_link_libraries(traj_2_ply ${catkin_LIBRARIES} )

add_executable(pcd_2_ply pcd_2_ply.cpp)
target_link_libraries(pcd_2_ply ${PCL_LIBRARIES} ${catkin_LIBRARIES})

################## generate 3d map incrementally for recording ##############
# add_executable(map_video map_video.cpp)
# target_link_libraries(map_video gtsam_graph vro sr4k_io ${OpenCV_LIBS})

# add_executable(map_video_mtraj map_video_mtraj.cpp)
# target_link_libraries(map_video_mtraj gtsam_graph vro sr4k_io ${OpenCV_LIBS})

# add_executable(map_video_rs map_video_rs.cpp)
# target_link_libraries(map_video_rs gtsam_graph ${OpenCV_LIBS}  ${PCL_LIBRARIES} ${catkin_LIBRARIES} )




