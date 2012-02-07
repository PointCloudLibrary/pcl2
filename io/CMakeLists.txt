set(SUBSYS_NAME io)
set(SUBSYS_DESC "libpcl2_io: I/O operations")
set(SUBSYS_DEPS core)

set(build TRUE)
PCL_SUBSYS_OPTION(build ${SUBSYS_NAME} ${SUBSYS_DESC} ON)
PCL_SUBSYS_DEPEND(build ${SUBSYS_NAME} DEPS ${SUBSYS_DEPS})

#PCL_ADD_DOC(${SUBSYS_NAME})

if(build)
    set(srcs 
        src/io.cpp
       )

    set(incs 
        include/pcl2/io/io.h
        )
        
    set(impl_incs 
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
