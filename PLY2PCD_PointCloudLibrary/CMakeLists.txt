file (GLOB SOURCE_FILE_LIST src/*.cpp)
file (GLOB HEADER_FILE_LIST src/*.h)

include(${VTK_USE_FILE})
link_directories(${VTK_LIBRARY_DIRS})

include_directories(${BOOST_INC} ${PCL_INC} ${EIGEN_INC} ${FLANN_INC})
link_directories(${BOOST_LIB} ${PCL_LIB} ${FLANN_LIB})

add_executable(PLY2PCD_PointCloudLibrary ${SOURCE_FILE_LIST} ${HEADER_FILE_LIST})
target_link_libraries(PLY2PCD_PointCloudLibrary QVTK pcl_visualization_debug pcl_registration_debug pcl_io_debug pcl_kdtree_debug pcl_common_debug pcl_search_debug)
