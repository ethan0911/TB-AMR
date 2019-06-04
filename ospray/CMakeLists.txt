include_directories ("../../../")
find_package(MPI REQUIRED)

# Use our Findp4est
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_CURRENT_LIST_DIR}/cmake")
find_package(p4est REQUIRED)

ospray_create_library(ospray_module_p4est
  P4estVolume.cpp
  P4estVolume.ispc
  VoxelOctree.cpp
  FindDualCell.ispc)

target_include_directories(ospray_module_p4est PRIVATE
  $<BUILD_INTERFACE:${P4EST_INCLUDE_DIR}>
  $<BUILD_INTERFACE:${MPI_CXX_INCLUDE_PATH}>)

target_link_libraries(ospray_module_p4est PUBLIC ospray_module_ispc)

