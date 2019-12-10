include_directories ("../../../")
find_package(MPI REQUIRED)


# prefer libGL over libOpenGl for better compatibility with SWR
set(OpenGL_GL_PREFERENCE "LEGACY")
find_package(OpenGL 2 REQUIRED)

find_package(glfw3 QUIET)
if (TARGET glfw)
  get_property(glfw3_INCLUDE_DIRS TARGET glfw
    PROPERTY INTERFACE_INCLUDE_DIRECTORIES
  )
endif()

set(OSPRAY_TUTORIALS_DIR "${OSPRay_SOURCE_DIR}/apps/tutorials")

option(OSPRAY_VTK "Enable VTK loaders" ON)

if(OSPRAY_VTK)
find_package(VTK REQUIRED)
  include(${VTK_USE_FILE})
  include_directories(${VTK_INCLUDE_DIRS})

  add_definitions(-DOSPRAY_APPS_VTK=1)
endif()



#############################################
######      VoxelOctree Build App       #####
#############################################
add_executable(ospRaw2Octree
  ospRaw2Octree.cpp
  dataImporter.cpp
  loader/meshloader.cpp
)

target_link_libraries(ospRaw2Octree
PRIVATE
  ospray
  ospcommon::ospcommon
  ospray_imgui
  ospray_module_tamr
  ${MPI_CXX_LIBRARIES}
  ${VTK_LIBRARIES}
  ${P4EST_LIBRARIES}
)

target_include_directories(ospRaw2Octree PRIVATE
  $<BUILD_INTERFACE:${OSPRAY_TUTORIALS_DIR}>
  $<BUILD_INTERFACE:${P4EST_INCLUDE_DIR}>
  $<BUILD_INTERFACE:${MPI_CXX_INCLUDE_PATH}>)



#############################################
######          TAMR DVR Viewer         #####
#############################################
add_executable(tamrViewer
  tamrViewer.cpp
  dataImporter.cpp
  loader/meshloader.cpp
  widgets/transfer_function_widget.cpp
  ${OSPRAY_TUTORIALS_DIR}/GLFWOSPRayWindow.cpp
  ${OSPRAY_TUTORIALS_DIR}/ArcballCamera.cpp
  ${OSPRAY_TUTORIALS_DIR}/imgui/imgui_impl_glfw_gl3.cpp
)

target_link_libraries(tamrViewer
PRIVATE
  ospray
  ospcommon::ospcommon
  ospray_imgui
  ospray_module_tamr
  ospray_module_impi
  glfw
  ${OPENGL_LIBRARIES}
  ${MPI_CXX_LIBRARIES}
  ${VTK_LIBRARIES}
  ${P4EST_LIBRARIES}
)

target_include_directories(tamrViewer PRIVATE
  $<BUILD_INTERFACE:${OSPRAY_TUTORIALS_DIR}>
  $<BUILD_INTERFACE:${P4EST_INCLUDE_DIR}>
  $<BUILD_INTERFACE:${MPI_CXX_INCLUDE_PATH}>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_LIST_DIR}/widgets/)
