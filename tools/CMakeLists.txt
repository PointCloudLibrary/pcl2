set (SUBSYS_NAME tools)
set (SUBSYS_DESC "Useful PCL-based command line tools")
set (SUBSYS_DEPS core io registration search)
set (DEFAULT ON)
set (REASON "")

PCL_SUBSYS_OPTION (build ${SUBSYS_NAME} ${SUBSYS_DESC} ${DEFAULT} ${REASON})
PCL_SUBSYS_DEPEND (build ${SUBSYS_NAME} DEPS ${SUBSYS_DEPS})

if (build)
  PCL_ADD_EXECUTABLE (mat_intro ${SUBSYS_NAME} mat_intro.cpp)
  target_link_libraries (mat_intro pcl2_core pcl2_io pcl2_search pcl2_registration ${PCL_LIBRARIES})

  PCL_ADD_EXECUTABLE (cloud_intro ${SUBSYS_NAME} cloud_intro.cpp)
  target_link_libraries (cloud_intro pcl2_core pcl2_io pcl2_search pcl2_registration ${PCL_LIBRARIES})

  PCL_ADD_EXECUTABLE (rst_test ${SUBSYS_NAME} rst_test.cpp)
  target_link_libraries (rst_test pcl2_core ${PCL_LIBRARIES})

  PCL_ADD_EXECUTABLE (pcl2_test ${SUBSYS_NAME} pcl2_test.cpp)
  target_link_libraries (pcl2_test pcl2_core pcl2_io pcl2_search pcl2_registration ${PCL_LIBRARIES})

endif ()
