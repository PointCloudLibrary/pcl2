set(SUBSYS_NAME core)
set(SUBSYS_DESC "Point cloud base library")
set(SUBSYS_DEPS)

set(build TRUE)
PCL_SUBSYS_OPTION(build ${SUBSYS_NAME} ${SUBSYS_DESC} ON)
PCL_SUBSYS_DEPEND(build ${SUBSYS_NAME} ${SUBSYS_DEPS})

if(build)
    set(srcs src/point_cloud.cpp
        )

    set(incs include/pcl/exception.h
        include/pcl/point_cloud.h
        include/pcl/point_types.h
        include/pcl/channel.h
        include/pcl/const_channel.h
        include/pcl/channel_base.h
        include/pcl/channel_impl.h
        include/pcl/win32_macros.h
        include/pcl/pcl_macros.h    
        )
        
#    if(NOT USE_ROS)
#        set(incs ${incs}
#          include/pcl/ModelCoefficients.h
#          include/pcl/PolygonMesh.h
#          include/pcl/Vertices.h
#          include/pcl/PointIndices.h
#        )
#
#        set(sensor_msg_incs 
#          include/sensor_msgs/PointField.h
#          include/sensor_msgs/PointCloud2.h
#          include/sensor_msgs/Image.h
#          )

#        set(std_msg_incs include/std_msgs/Header.h)
#    endif(NOT USE_ROS)
    
    set(common_incs
#        include/pcl/common/angles.h
#        include/pcl/common/bivariate_polynomial.h
#        include/pcl/common/centroid.h
#        include/pcl/common/concatenate.h
#        include/pcl/common/common.h
#        include/pcl/common/common_headers.h
#        include/pcl/common/distances.h
#        include/pcl/common/eigen.h
#        include/pcl/common/file_io.h
#        include/pcl/common/intersections.h
#        include/pcl/common/norms.h
#        include/pcl/common/piecewise_linear_function.h
#        include/pcl/common/point_correspondence.h
#        include/pcl/common/polynomial_calculations.h
#        include/pcl/common/poses_from_matches.h
#        include/pcl/common/rigid_transforms.h
        include/pcl/common/time.h
#        include/pcl/common/time_trigger.h
#        include/pcl/common/transform.h
#        include/pcl/common/transformation_from_correspondences.h
#        include/pcl/common/vector_average.h
#        include/pcl/common/pca.h
        include/pcl/common/synchronizer.h
        )

#    set(common_incs_impl
#        include/pcl/common/impl/angles.hpp
#        include/pcl/common/impl/bivariate_polynomial.hpp
#        include/pcl/common/impl/centroid.hpp
#        include/pcl/common/impl/common.hpp
#        include/pcl/common/impl/file_io.hpp
#        include/pcl/common/impl/norms.hpp
#        include/pcl/common/impl/piecewise_linear_function.hpp
#        include/pcl/common/impl/polynomial_calculations.hpp
#        include/pcl/common/impl/rigid_transforms.hpp
#        include/pcl/common/impl/pca.hpp
#        include/pcl/common/impl/transform.hpp
#        include/pcl/common/impl/transformation_from_correspondences.hpp
#        include/pcl/common/impl/vector_average.hpp
#       )

#    set(impl_incs include/pcl/impl/instantiate.hpp
#        include/pcl/impl/point_types.hpp
#        )

#    set(ros_incs include/pcl/ros/conversions.h
#        include/pcl/ros/for_each_type.h
#        include/pcl/ros/point_traits.h
#        include/pcl/ros/register_point_struct.h
#        )

#    set(tools_incs 
#        include/pcl/console/print.h
#        include/pcl/console/ArgException.h
#        include/pcl/console/Arg.h
#        include/pcl/console/ArgTraits.h
#        include/pcl/console/CmdLine.h
#        include/pcl/console/CmdLineInterface.h
#        include/pcl/console/CmdLineOutput.h
#        include/pcl/console/Constraint.h
#        include/pcl/console/DocBookOutput.h
#        include/pcl/console/HelpVisitor.h
#        include/pcl/console/IgnoreRestVisitor.h
#        include/pcl/console/MultiArg.h
#        include/pcl/console/MultiSwitchArg.h
#        include/pcl/console/OptionalUnlabeledTracker.h
#        include/pcl/console/print.h
#        include/pcl/console/StandardTraits.h
#        include/pcl/console/StdOutput.h
#        include/pcl/console/SwitchArg.h
#        include/pcl/console/UnlabeledMultiArg.h
#        include/pcl/console/UnlabeledValueArg.h
#        include/pcl/console/ValueArg.h
#        include/pcl/console/ValuesConstraint.h
#        include/pcl/console/VersionVisitor.h
#        include/pcl/console/Visitor.h
#        include/pcl/console/XorHandler.h
#        include/pcl/console/ZshCompletionOutput.h
#        )


    set(LIB_NAME pcl_${SUBSYS_NAME})
    include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)
    PCL_ADD_LIBRARY(${LIB_NAME} ${SUBSYS_NAME} ${srcs} ${incs} ${common_incs} ${impl_incs} ${ros_incs} ${tools_incs} ${common_incs_impl} ${sensor_msg_incs} ${std_msg_incs})
    link_ros_libs (${LIB_NAME})
    PCL_ADD_SSE_FLAGS(${LIB_NAME})
    PCL_MAKE_PKGCONFIG(${LIB_NAME} ${SUBSYS_NAME} "${SUBSYS_DESC}" "" ""
        "" "")

    # Install include files
    PCL_ADD_INCLUDES(${SUBSYS_NAME} "" ${incs})
    if(NOT USE_ROS)
        PCL_ADD_INCLUDES(${SUBSYS_NAME} "../sensor_msgs" ${sensor_msg_incs})
        PCL_ADD_INCLUDES(${SUBSYS_NAME} "../std_msgs" ${std_msg_incs})
    endif(NOT USE_ROS)
#    PCL_ADD_INCLUDES(${SUBSYS_NAME} common ${common_incs})
#    PCL_ADD_INCLUDES(${SUBSYS_NAME} common/impl ${common_incs_impl})
#    PCL_ADD_INCLUDES(${SUBSYS_NAME} impl ${impl_incs})
    PCL_ADD_INCLUDES(${SUBSYS_NAME} ros ${ros_incs})
    PCL_ADD_INCLUDES(${SUBSYS_NAME} console ${tools_incs})

#    add_subdirectory(test)
endif(build)
