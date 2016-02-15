SET( EFSLAM_IO_SRSC
    carmen/CarmenReader.cpp
    carmen/CarmenGtReader.cpp
)

IF( catkin_FOUND )
    ADD_SUBDIRECTORY( ros )
ENDIF()

ADD_LIBRARY( efslam-io SHARED ${EFSLAM_IO_SRSC} )
#ADD_LIBRARY( efslam-io-static STATIC ${EFSLAM_IO_SRSC} )

IF( catkin_FOUND )
    target_link_libraries(efslam-io efslam-ros)
    #target_link_libraries(efslam-io-static efslam-ros-static)
ENDIF()

ADD_EXECUTABLE( EFSlamCarmenGui carmen/EFSlamCarmenGui.cpp )
TARGET_LINK_LIBRARIES( EFSlamCarmenGui efslam efslam-viz ${PCL_LIBRARIES})