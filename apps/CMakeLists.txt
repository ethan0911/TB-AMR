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

# transfer function widget
set(TFN_MODULE_DEFAULT_ROOT 
  ${PROJECT_SOURCE_DIR}/apps/exampleViewer/widgets/TransferFunctionModule) 
if(TFN_MODULE_ROOT)
  set(TFN_MODULE 1)
elseif (EXISTS ${TFN_MODULE_DEFAULT_ROOT})
  set(TFN_MODULE_ROOT ${TFN_MODULE_DEFAULT_ROOT})
  set(TFN_MODULE 1)
else ()
  set(TFN_MODULE 0)
endif ()
if (TFN_MODULE)
  set(TFN_MODULE_INCLUDE_DIR ${TFN_MODULE_ROOT}/src)
  set(TFN_MODULE_SRC 
    ${TFN_MODULE_ROOT}/src/TransferFunctionModule.cpp
    ${TFN_MODULE_ROOT}/src/widgets/TransferFunctionWidget.cpp)
endif (TFN_MODULE)


option(OSPRAY_VTK "Enable VTK loaders" ON)

if(OSPRAY_VTK)
find_package(VTK REQUIRED)
  include(${VTK_USE_FILE})
  include_directories(${VTK_INCLUDE_DIRS})

  add_definitions(-DOSPRAY_APPS_VTK=1)
endif()



ospray_create_application(tamrViewer
  tamrViewer.cpp
  dataImporter.cpp
  loader/meshloader.cpp
  ${OSPRAY_TUTORIALS_DIR}/GLFWOSPRayWindow.cpp
  ${OSPRAY_TUTORIALS_DIR}/ArcballCamera.cpp
  ${OSPRAY_TUTORIALS_DIR}/imgui/imgui_impl_glfw_gl3.cpp
  ${TFN_MODULE_SRC}
)

target_link_libraries(tamrViewer
PRIVATE
  ospray
  ospray_common
  ospray_imgui
  ospray_module_tamr
  glfw
  ${OPENGL_LIBRARIES}
  ${MPI_CXX_LIBRARIES}
  ${VTK_LIBRARIES}
  ${P4EST_LIBRARIES}
)

target_include_directories(tamrViewer PRIVATE
  $<BUILD_INTERFACE:${OSPRAY_TUTORIALS_DIR}>
  $<BUILD_INTERFACE:${TFN_MODULE_INCLUDE_DIR}>
  $<BUILD_INTERFACE:${P4EST_INCLUDE_DIR}>
  $<BUILD_INTERFACE:${MPI_CXX_INCLUDE_PATH}>)


ospray_create_application(ospRaw2Octree
  ospRaw2Octree.cpp
  dataImporter.cpp
  loader/meshloader.cpp
)

target_link_libraries(ospRaw2Octree
PRIVATE
  ospray
  ospray_common
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


# The distributed p4est viewer for data-parallel rendering
# ospray_create_application(p4estDistributedViewer
#   p4estDistributedViewer.cpp
#   GLFWDistribP4estWindow.cpp
#   ${OSPRAY_TUTORIALS_DIR}/ArcballCamera.cpp
#   ${OSPRAY_TUTORIALS_DIR}/imgui/imgui_impl_glfw_gl3.cpp
# )

# target_link_libraries(p4estDistributedViewer
# PRIVATE
#   ospray
#   ospray_common
#   ospray_imgui
#   ospray_module_p4est
#   glfw
#   ${OPENGL_LIBRARIES}
#   ${MPI_CXX_LIBRARIES}
#   ${P4EST_LIBRARIES}
# )

# target_include_directories(p4estDistributedViewer PRIVATE
#   $<BUILD_INTERFACE:${OSPRAY_TUTORIALS_DIR}>
#   $<BUILD_INTERFACE:${P4EST_INCLUDE_DIR}>
#   $<BUILD_INTERFACE:${MPI_CXX_INCLUDE_PATH}>)

