file (GLOB SOURCE_FILE_LIST src/*.cpp)
file (GLOB HEADER_FILE_LIST src/*.h)

add_executable(Scanning3D_ReconstructMe ${SOURCE_FILE_LIST} ${HEADER_FILE_LIST})
