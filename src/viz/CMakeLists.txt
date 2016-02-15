SET( EFSLAM_VIZ_SRSC
    MapView2D.cpp
    NeffView.cpp
)

SET( EFSLAM_VIZ_INCLUDE_DIR ../../include/efslam/viz )

SET( EFSLAM_VIZ_MOC_HDRS
    ${EFSLAM_VIZ_INCLUDE_DIR}/MapView2D.h
    ${EFSLAM_VIZ_INCLUDE_DIR}/NeffView.h
)

QT4_WRAP_CPP( EFSLAM_VIZ_MOC_SRCS ${EFSLAM_VIZ_MOC_HDRS} )

ADD_LIBRARY( efslam-viz SHARED ${EFSLAM_VIZ_SRSC} ${EFSLAM_VIZ_MOC_SRCS} )
#ADD_LIBRARY( efslam-viz-static STATIC ${EFSLAM_VIZ_SRSC} ${EFSLAM_VIZ_MOC_SRCS} )

target_link_libraries( efslam-viz ${VTK_LIBRARIES} ${QT_LIBRARIES} )
#target_link_libraries( efslam-viz-static ${VTK_LIBRARIES} ${QT_LIBRARIES} )

