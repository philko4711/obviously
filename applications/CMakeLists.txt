cmake_minimum_required (VERSION 2.6)
include(FindPkgConfig)

############################
# General project settings #
############################

PROJECT(APPLICATIONS)
SET(APPLICATIONS_VERSION_MAJOR 0)
SET(APPLICATIONS_VERSION_MINOR 1)

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})
PKG_CHECK_MODULES(XML libxml++-2.6)

INCLUDE_DIRECTORIES(/usr/include/openni2
                    /usr/include/ni
					/usr/include/openni
                    $ENV{OBVIOUSLY_ROOT}
                    /usr/local/include/
                    /usr/include/qt4/
                    /usr/include/lua5.1
                    /opt/ros/electric/stacks/perception_pcl/flann/include/
                    ../obcore/math
                    ../obcore/filter
                    ../obcore
                    ${XML_INCLUDE_DIRS}
                    /usr/include/eigen3
                    )

SET(CORELIBS    obcore    gsl gslcblas lua5.1)
SET(DEVICELIBS  obdevice  OpenNI v4l2 udev )
SET(GRAPHICLIBS obgraphic ${VTK_LIBRARIES} glut GL jpeg)
SET(VISIONLIBS  obvision  ann flann)

# check if necessary files for PMD are installed
if(EXISTS /usr/local/include/PMDSDK)
  MESSAGE("Found PMDSDK. Will build driver for CamBoard Nano.")
  SET(DEVICELIBS ${DEVICELIBS} pmdaccess2)
	INCLUDE_DIRECTORIES(/usr/local/include/PMDSDK)
	LINK_DIRECTORIES(/usr/local/lib/PMDSDK)
	ADD_EXECUTABLE(nanoStream                     nanoStream.cpp)
	ADD_EXECUTABLE(tsd_camboardNano               tsd_camboardNano.cpp)
	TARGET_LINK_LIBRARIES(nanoStream              ${VISIONLIBS} ${GRAPHICLIBS} ${DEVICELIBS} ${CORELIBS} ${XML_LIBRARIES})
	TARGET_LINK_LIBRARIES(tsd_camboardNano        ${VISIONLIBS}  ${DEVICELIBS}  ${GRAPHICLIBS} ${CORELIBS} ${XML_LIBRARIES})
endif()

if(EXISTS /usr/local/include/LMS1xx.h)
  MESSAGE("Found libLMS. Will build drivers for Sick Laser Scanner.")
  SET(DEVICELIBS ${DEVICELIBS} LMS1xx)
  ADD_EXECUTABLE(tsd_grid_lms100 tsd_grid_lms100.cpp)
  TARGET_LINK_LIBRARIES(tsd_grid_lms100 ${VISIONLIBS} ${GRAPHICLIBS} ${DEVICELIBS} ${CORELIBS})
endif()

# if does exists find libopenni2 directories and libs
EXECUTE_PROCESS(COMMAND pkg-config libopenni2 --exists RESULT_VARIABLE RET_OPENNI_CODE)
IF("${RET_OPENNI_CODE}" STREQUAL "0")
    PKG_CHECK_MODULES(OPENNI libopenni2)
    INCLUDE_DIRECTORIES(${OPENNI_INCLUDE_DIRS})
    LINK_DIRECTORIES(${OPENNI_LIBRARY_DIRS})
    ADD_EXECUTABLE(xtionStream                xtionStream.cpp)
	SET_TARGET_PROPERTIES(xtionStream PROPERTIES COMPILE_FLAGS "-Wno-unknown-pragmas -Wno-reorder")	
	ADD_EXECUTABLE(tsd_xtion                  tsd_xtion.cpp)
	SET_TARGET_PROPERTIES(tsd_xtion PROPERTIES COMPILE_FLAGS "-Wno-unknown-pragmas -Wno-reorder")	
    TARGET_LINK_LIBRARIES(xtionStream              ${VISIONLIBS}  ${GRAPHICLIBS} ${DEVICELIBS} ${CORELIBS} ${XML_LIBRARIES} ${OPENNI_LIBRARIES})
    TARGET_LINK_LIBRARIES(tsd_xtion                ${VISIONLIBS}  ${DEVICELIBS}  ${GRAPHICLIBS} ${CORELIBS} ${XML_LIBRARIES} ${OPENNI_LIBRARIES})
ENDIF()

LINK_DIRECTORIES(/opt/ros/electric/stacks/perception_pcl/flann/lib64/
                 /usr/local/lib
                 ${OPENNI_LIBRARY_DIRS}
                 )

ADD_EXECUTABLE(obvious3D_show             obvious3D_show.cpp)
ADD_EXECUTABLE(kinect_perspective         kinect_perspective.cpp)
ADD_EXECUTABLE(icp_matching2D             icp_matching2D.cpp)
ADD_EXECUTABLE(ransac_matching2D          ransac_matching2D.cpp)
ADD_EXECUTABLE(ndt_matching2D             ndt_matching2D.cpp)
ADD_EXECUTABLE(icp_interactive3D          icp_interactive3D.cpp)
ADD_EXECUTABLE(kinect_stream_show         kinect_stream_show.cpp)
SET_TARGET_PROPERTIES(kinect_stream_show PROPERTIES COMPILE_FLAGS "-Wno-unknown-pragmas -Wno-reorder")
ADD_EXECUTABLE(kinect_mesh_show           kinect_mesh_show.cpp)
SET_TARGET_PROPERTIES(kinect_mesh_show PROPERTIES COMPILE_FLAGS "-Wno-unknown-pragmas -Wno-reorder")
ADD_EXECUTABLE(kinect_playback            kinect_playback.cpp)
ADD_EXECUTABLE(kinect_localize            kinect_localize.cpp)
SET_TARGET_PROPERTIES(kinect_localize PROPERTIES COMPILE_FLAGS "-Wno-unknown-pragmas -Wno-reorder")
ADD_EXECUTABLE(uvccam_serialize           uvccam_serialize.cpp)
ADD_EXECUTABLE(uvcvirtualcam_serialize    uvcvirtualcam_serialize.cpp)
ADD_EXECUTABLE(uvccam_stream              uvccam_stream.cpp)
ADD_EXECUTABLE(uvccam_querycapabilities   uvccam_querycapabilities.cpp)
ADD_EXECUTABLE(uvccam_finddevice          uvccam_finddevice.cpp)
ADD_EXECUTABLE(synthetic_pointcloud       synthetic_pointcloud.cpp)
ADD_EXECUTABLE(lua_read_config            lua_read_config.cpp)
ADD_EXECUTABLE(lua_call_function          lua_call_function.cpp)
ADD_EXECUTABLE(lua_callback_c             lua_callback_c.cpp)
ADD_EXECUTABLE(lua_statemachine           lua_statemachine.cpp)
ADD_EXECUTABLE(logging_example            logging_example.cpp)
ADD_EXECUTABLE(tsd_test                   tsd_test.cpp)
ADD_EXECUTABLE(tsd_grid_test              tsd_grid_test.cpp)
ADD_EXECUTABLE(tsd_kinect                 tsd_kinect.cpp)
ADD_EXECUTABLE(tsd_testMineShaft          tsd_testMineShaft.cpp)
ADD_EXECUTABLE(astar_test                 astar_test.cpp)
ADD_EXECUTABLE(statemachine_test          statemachine_test.cpp)
ADD_EXECUTABLE(ransac_cirle               ransac_circle.cpp)

SET_TARGET_PROPERTIES(tsd_kinect PROPERTIES COMPILE_FLAGS "-Wno-unknown-pragmas -Wno-reorder")
ADD_EXECUTABLE(tsd_raycast_visualize      tsd_raycast_visualize.cpp)
ADD_EXECUTABLE(showCloud                  showCloud.cpp)


TARGET_LINK_LIBRARIES(obvious3D_show           ${GRAPHICLIBS} ${CORELIBS})
TARGET_LINK_LIBRARIES(kinect_perspective       ${GRAPHICLIBS} ${CORELIBS})
TARGET_LINK_LIBRARIES(icp_matching2D           ${VISIONLIBS}  ${CORELIBS} ${XML_LIBRARIES})
TARGET_LINK_LIBRARIES(ransac_matching2D        ${VISIONLIBS}  ${CORELIBS} ${XML_LIBRARIES})
TARGET_LINK_LIBRARIES(ndt_matching2D           ${VISIONLIBS}  ${CORELIBS} ${XML_LIBRARIES})
TARGET_LINK_LIBRARIES(icp_interactive3D        ${GRAPHICLIBS} ${VISIONLIBS} ${CORELIBS})
TARGET_LINK_LIBRARIES(kinect_stream_show       ${VISIONLIBS}  ${GRAPHICLIBS} ${DEVICELIBS} ${CORELIBS} ${XML_LIBRARIES})
TARGET_LINK_LIBRARIES(kinect_mesh_show         ${VISIONLIBS}  ${GRAPHICLIBS} ${DEVICELIBS} ${CORELIBS} ${XML_LIBRARIES})
TARGET_LINK_LIBRARIES(kinect_playback          ${GRAPHICLIBS} ${DEVICELIBS} ${CORELIBS})
TARGET_LINK_LIBRARIES(kinect_localize          ${VISIONLIBS}  ${GRAPHICLIBS} ${DEVICELIBS} ${CORELIBS} ${XML_LIBRARIES})
TARGET_LINK_LIBRARIES(uvccam_serialize         ${DEVICELIBS}  ${CORELIBS})
TARGET_LINK_LIBRARIES(uvcvirtualcam_serialize  ${DEVICELIBS}  ${CORELIBS})
TARGET_LINK_LIBRARIES(uvccam_stream            ${GRAPHICLIBS} ${DEVICELIBS} ${CORELIBS})
TARGET_LINK_LIBRARIES(uvccam_querycapabilities ${DEVICELIBS}  ${CORELIBS})
TARGET_LINK_LIBRARIES(uvccam_finddevice        ${DEVICELIBS}  ${CORELIBS})
TARGET_LINK_LIBRARIES(synthetic_pointcloud     ${GRAPHICLIBS} ${CORELIBS})
TARGET_LINK_LIBRARIES(lua_read_config          ${CORELIBS})
TARGET_LINK_LIBRARIES(lua_call_function        ${CORELIBS})
TARGET_LINK_LIBRARIES(lua_callback_c           ${CORELIBS})
TARGET_LINK_LIBRARIES(lua_statemachine         ${CORELIBS})
TARGET_LINK_LIBRARIES(logging_example          ${CORELIBS})
TARGET_LINK_LIBRARIES(tsd_test                 ${VISIONLIBS}  ${GRAPHICLIBS} ${CORELIBS})
TARGET_LINK_LIBRARIES(tsd_grid_test            ${VISIONLIBS}  ${GRAPHICLIBS} ${CORELIBS})
TARGET_LINK_LIBRARIES(tsd_kinect               ${VISIONLIBS}  ${DEVICELIBS}  ${GRAPHICLIBS} ${CORELIBS} ${XML_LIBRARIES})
TARGET_LINK_LIBRARIES(tsd_raycast_visualize    ${VISIONLIBS}  ${GRAPHICLIBS} ${CORELIBS})
TARGET_LINK_LIBRARIES(tsd_testMineShaft        ${VISIONLIBS}  ${GRAPHICLIBS} ${CORELIBS})
TARGET_LINK_LIBRARIES(showCloud                ${GRAPHICLIBS} ${CORELIBS})
TARGET_LINK_LIBRARIES(astar_test               ${VISIONLIBS}  ${CORELIBS})
TARGET_LINK_LIBRARIES(statemachine_test        ${VISIONLIBS}  ${CORELIBS})
TARGET_LINK_LIBRARIES(ransac_cirle             ${VISIONLIBS}  ${CORELIBS})

