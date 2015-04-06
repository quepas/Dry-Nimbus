file (GLOB SOURCE_FILE_LIST src/*.cpp)
file (GLOB HEADER_FILE_LIST src/*.h)

include_directories(${VCG_INC} ${VCG_WRAP_INC})
#link_directories(${BOOST_LIB} ${PCL_LIB} ${FLANN_LIB})

add_executable(PLYMeshDifference_VCGLibrary ${SOURCE_FILE_LIST} ${HEADER_FILE_LIST})
#target_link_libraries(PLYMeshDifference_VCGLibrary)
