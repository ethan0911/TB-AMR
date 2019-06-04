option(OSPRAY_MODULE_P4EST "Enable the p4est module" OFF)
if (OSPRAY_MODULE_P4EST)
  add_subdirectory(ospray)
  add_subdirectory(apps)
endif()

