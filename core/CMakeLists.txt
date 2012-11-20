set(SUBSYS_NAME core)
set(SUBSYS_DESC "libpcl2_core: core structures")
set(SUBSYS_DEPS)

set(build TRUE)
PCL_SUBSYS_OPTION(build ${SUBSYS_NAME} ${SUBSYS_DESC} ON)
PCL_SUBSYS_DEPEND(build ${SUBSYS_NAME} DEPS ${SUBSYS_DEPS} EXT_DEPS eigen boost)

#PCL_ADD_DOC(${SUBSYS_NAME})

if(build)
    set(srcs 
        src/cloud.cpp
        src/matrix.cpp
	src/row.cpp
        src/eigen_matrix.cpp
#        src/conversions.cpp
	src/math.cpp
	src/stats.cpp
	#src/create.cpp
       )

    set(incs 
        include/pcl2/cloud.h
#        include/pcl2/conversions.h
	include/pcl2/core.h
	include/pcl2/create.h
        include/pcl2/eigen_matrix.h
        include/pcl2/eigen_matrix_impl.h
        include/pcl2/exception.h
        include/pcl2/kernels.h
	include/pcl2/math.h
        include/pcl2/matrix.h
        include/pcl2/matrix_impl.h
        include/pcl2/matrix_row_impl.h
        include/pcl2/matrix_view_impl.h
        include/pcl2/row.h
	include/pcl2/spatial_index.h
	include/pcl2/stats.h
        include/pcl2/typed_matrix.h
        include/pcl2/typed_matrix_impl.h
        )
        
    set(impl_incs 
        include/pcl2/impl/eigen_matrix.hpp
        include/pcl2/impl/eigen_matrix_impl.hpp
	include/pcl2/impl/math.hpp
        include/pcl2/impl/matrix_row_impl.hpp
        include/pcl2/impl/matrix_view_impl.hpp
	include/pcl2/impl/row.hpp
	include/pcl2/impl/stats.hpp
        include/pcl2/impl/typed_matrix.hpp
	#include/pcl2/impl/create.hpp
        )

    set(LIB_NAME pcl2_${SUBSYS_NAME})
    include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)
    PCL_ADD_LIBRARY(${LIB_NAME} ${SUBSYS_NAME} ${srcs} ${incs} ${impl_incs})
    PCL_ADD_SSE_FLAGS(${LIB_NAME})
    PCL_MAKE_PKGCONFIG(${LIB_NAME} ${SUBSYS_NAME} "${SUBSYS_DESC}" "" "" "" "" "")

    # Install include files
    PCL_ADD_INCLUDES(${SUBSYS_NAME} "" ${incs})
    PCL_ADD_INCLUDES(${SUBSYS_NAME} impl ${impl_incs})

    #if(BUILD_TESTS)
    #   add_subdirectory(test)
    #endif(BUILD_TESTS)

    #if(BUILD_EXAMPLES)
    #   add_subdirectory(examples)
    #endif(BUILD_EXAMPLES)
endif(build)
