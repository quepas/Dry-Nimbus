file (GLOB SOURCE_FILE_LIST src/*.cpp)
file (GLOB HEADER_FILE_LIST src/*.h)

set (VCG_SOURCE "${VCG_WRAP_INC}/wrap/ply/plylib.cpp")

include_directories(${VCG_INC} ${VCG_WRAP_INC})

add_executable(PLYMeshDifference_VCGLibrary ${SOURCE_FILE_LIST} ${HEADER_FILE_LIST} ${VCG_SOURCE})
