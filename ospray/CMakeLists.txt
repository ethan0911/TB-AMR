include_directories ("../../../")
find_package(MPI REQUIRED)

# Use our Findp4est
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_CURRENT_LIST_DIR}/cmake")
find_package(p4est REQUIRED)

add_library(ospray_module_tamr SHARED)

ispc_target_add_sources(ospray_module_tamr
  TAMRVolume.cpp
  TAMRVolume.ispc
  VoxelOctree.cpp
  FindDualCell.ispc
  filter_nearest.ispc
  filter_current.ispc
  filter_finest.ispc
  filter_octant.ispc
  filter_trilinear.ispc)

target_include_directories(ospray_module_tamr PRIVATE
  $<BUILD_INTERFACE:${P4EST_INCLUDE_DIR}>
  $<BUILD_INTERFACE:${MPI_CXX_INCLUDE_PATH}>)

target_link_libraries(ospray_module_tamr PUBLIC ospray_module_ispc)

add_library(ospray_module_impi SHARED)

ispc_target_add_sources(ospray_module_impi
  # the cpp file that contains all the plugin code - parsing
  # parameters in ospCommit(), creating and registering the object,
  # building accel strcutures, etc
  isosurface/geometry/Impi.cpp
  
  # the ispc file that contains the plugins for all vector code - ie,
  # for ray-primitive intersection and 'postIntersect' (reporting info
  # on a previously computed ray-prim intersection)
  isosurface/geometry/Impi.ispc

  # =======================================================
  # "instantiations" of the Impi abstractin: ie, class that can
  # generate voxels that Impi can then build a bvh over and intersct
  # testamr - generates a simple, amr-like test data set
  isosurface/voxelSources/testCase/TestAMR.cpp
  isosurface/voxelSources/testCase/TestTAMR.cpp
  isosurface/voxelSources/testCase/TestOctant.cpp
  isosurface/voxelSources/testCase/compute_voxels.ispc
  # structuredvolume: generates (on the fly) all active voxels from a structured volume
  isosurface/voxelSources/structured/Volume.cpp
  isosurface/voxelSources/structured/StructuredVolumeSource.cpp
  # segmented volume: a structured volume, with an additiional
  # 'segmentatiln' volume that serves as a mask (only voxels whose
  # range in the segmentation volume overlap a given segment value
  # will be included
  isosurface/voxelSources/structured/SegmentedVolumeSource.cpp  
)

target_include_directories(ospray_module_impi PRIVATE
  $<BUILD_INTERFACE:${P4EST_INCLUDE_DIR}>
  $<BUILD_INTERFACE:${MPI_CXX_INCLUDE_PATH}>)

target_include_directories(ospray_module_impi PUBLIC ${TBB_INCLUDE_DIR})

target_link_libraries(ospray_module_impi PUBLIC ospray_module_ispc)
target_link_libraries(ospray_module_impi PUBLIC ${TBB_LIBRARY})
