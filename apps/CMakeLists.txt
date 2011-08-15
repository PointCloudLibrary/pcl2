set(SUBSYS_NAME apps)
set(SUBSYS_DESC "Application examples/samples that show how PCL works")
set(SUBSYS_DEPS core io)

if(NOT wxWidgets_FOUND)
  set(DEFAULT TRUE)   # wxWidgets is optional
  set(REASON "wxWidgets was not found.")
else(NOT wxWidgets_FOUND)
  include(${wxWidgets_USE_FILE})
  include_directories(${wxWidgets_INCLUDE_DIRS})
endif(NOT wxWidgets_FOUND)

# Find VTK 
if(NOT VTK_FOUND)
    set(DEFAULT FALSE)
    set(REASON "VTK was not found.")
else(NOT VTK_FOUND)
    set(DEFAULT TRUE)
    set(REASON)
    include (${VTK_USE_FILE})
endif(NOT VTK_FOUND)

# OpenNI found?
if(NOT OPENNI_FOUND)
    set(DEFAULT FALSE)
    set(REASON "OpenNI was not found.")
else(NOT OPENNI_FOUND)
    set(DEFAULT TRUE)
    set(REASON)
endif(NOT OPENNI_FOUND)

PCL_SUBSYS_OPTION(build ${SUBSYS_NAME} ${SUBSYS_DESC} ${DEFAULT} ${REASON})
PCL_SUBSYS_DEPEND(build ${SUBSYS_NAME} ${SUBSYS_DEPS})

if(build)

  include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)

  # Install include files
  PCL_ADD_INCLUDES(${SUBSYS_NAME} ${SUBSYS_NAME} ${incs})
  PCL_ADD_INCLUDES(${SUBSYS_NAME} ${SUBSYS_NAME}/impl ${impl_incs})
  
  PCL_ADD_EXECUTABLE(example ${SUBSYS_NAME} src/example.cpp)
  target_link_libraries(example pcl_core)

  if (VTK_FOUND AND OPENNI_FOUND)
    PCL_ADD_EXECUTABLE(openni_example ${SUBSYS_NAME} src/openni_example.cpp)
    target_link_libraries(openni_example pcl_core pcl_io)    
  endif()

endif(build)
