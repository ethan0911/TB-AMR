include_directories ("../../../")
find_package(MPI REQUIRED)

# Use our Findp4est
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_CURRENT_LIST_DIR}/cmake")
find_package(p4est REQUIRED)

ospray_create_library(ospray_module_tamr
  TAMRVolume.cpp
  TAMRVolume.ispc
  VoxelOctree.cpp
  FindDualCell.ispc)

target_include_directories(ospray_module_tamr PRIVATE
  $<BUILD_INTERFACE:${P4EST_INCLUDE_DIR}>
  $<BUILD_INTERFACE:${MPI_CXX_INCLUDE_PATH}>)

target_link_libraries(ospray_module_tamr PUBLIC ospray_module_ispc)

