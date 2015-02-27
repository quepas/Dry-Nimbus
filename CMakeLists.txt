cmake_minimum_required (VERSION 2.8)
project (Dry-Nimbus)

find_package(VTK REQUIRED)
## !! set this paths 	
set(BOOST_INC "D:/Programowanie/Biblioteki/boost_1.50_msvc2010_x86/include")
set(BOOST_LIB "D:/Programowanie/Biblioteki/boost_1.50_msvc2010_x86/lib")
set(EIGEN_INC "D:/Programowanie/Biblioteki/Tidy/Eigen/include")
set(FLANN_INC "D:/Programowanie/Biblioteki/Tidy/flann/include")
set(FLANN_LIB "D:/Programowanie/Biblioteki/Tidy/flann/lib")
set(PCL_INC "D:/Programowanie/Biblioteki/Tidy/pcl-1.7/include/pcl-1.7/")
set(PCL_LIB "D:/Programowanie/Biblioteki/Tidy/pcl-1.7/lib")
set(REME_INC "D:/Programy/RecounstructMe SDK/inc")
set(REME_LIB "D:/Programy/RecounstructMe SDK/lib")

add_subdirectory (RegistrationICP_PointCloudLibrary)
add_subdirectory (Scanning3D_ReconstructMe)
add_subdirectory (Visualization_ReconstructMe)
add_subdirectory (VisualizationMesh_PointCloudLibrary)
