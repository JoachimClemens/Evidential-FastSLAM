SET( EFSLAM_UTILS_SRSC
    Config.cpp
    Convenience.cpp
    Version.cpp
)

ADD_CUSTOM_COMMAND( OUTPUT VersionInfo.h
    COMMAND ${CMAKE_CURRENT_SOURCE_DIR}/../../scripts/create_version_info.sh ${CMAKE_CURRENT_SOURCE_DIR} Version.cpp
)
ADD_CUSTOM_TARGET( VersionInfo DEPENDS VersionInfo.h )

ADD_LIBRARY( efslam-utils SHARED ${EFSLAM_UTILS_SRSC} )
#ADD_LIBRARY( efslam-utils-static STATIC ${EFSLAM_UTILS_SRSC} )

ADD_DEPENDENCIES( efslam-utils VersionInfo )
#ADD_DEPENDENCIES( efslam-utils-static VersionInfo )

target_link_libraries( efslam-utils  ${EFSLAM_ROS_LIBS} )
#target_link_libraries( efslam-utils-static  ${EFSLAM_ROS_LIBS} )