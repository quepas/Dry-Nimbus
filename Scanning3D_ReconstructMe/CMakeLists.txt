file (GLOB SOURCE_FILE_LIST src/*.cpp)
file (GLOB HEADER_FILE_LIST src/*.h)

include_directories(${REME_INC})
link_directories(${REME_LIB})

add_executable(Scanning3D_ReconstructMe ${SOURCE_FILE_LIST} ${HEADER_FILE_LIST})
target_link_libraries(Scanning3D_ReconstructMe LibReconstructMeSDK)
