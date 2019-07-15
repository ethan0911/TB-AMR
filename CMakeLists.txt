option(OSPRAY_MODULE_TAMR "Enable the p4est module" ON)
if (OSPRAY_MODULE_TAMR)
  add_subdirectory(ospray)
  add_subdirectory(apps)
endif()

