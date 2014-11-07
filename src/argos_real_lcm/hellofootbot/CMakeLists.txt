set(hellofootbot_HEADERS
  includes/beans/node.h
  includes/lcm/lcmhandler.h
  includes/lcm/lcmthread.h
  includes/lcm/lcmtypes/lcmmsgs.hpp
  includes/lcm/lcmtypes/poselcm_pose_list_t.h
  includes/lcm/lcmtypes/poselcm_pose_t.h
  includes/lcm/lcmtypes/poselcm/pose_list_t.hpp
  includes/lcm/lcmtypes/poselcm/pose_t.hpp
  includes/thread/threadengine.h
  hellofootbot.h
)

set(hellofootbot_IMPLEMENTATIONS
  includes/beans/node.cpp
  includes/lcm/lcmhandler.cpp
  includes/lcm/lcmthread.cpp
  includes/thread/threadengine.cpp
  hellofootbot.cc
  
)

if( ARGOS_COMPILE_REAL_SWARMANOID )
  include_directories(/usr/local/angstrom/arm/include/)

  link_directories(
    ${CMAKE_BINARY_DIR}/../../../common/swarmanoid/control_interface
    ${CMAKE_BINARY_DIR}/../../../common/swarmanoid/utility
    ${CMAKE_BINARY_DIR}/../../../real_robot/swarmanoid/common
    ${CMAKE_BINARY_DIR}/../../../real_robot/swarmanoid/vision_robot/filtering
    ${CMAKE_SOURCE_DIR}/../../real_robot/swarmanoid/opencv/lib
    ${CMAKE_BINARY_DIR}/../../../real_robot/swarmanoid/footbot
  )
  add_executable(hellofootbot
    ${hellofootbot_IMPLEMENTATIONS}
    ${CMAKE_SOURCE_DIR}/main_functions/real_footbot_main.cpp
  )
  
  #ADD_LIBRARY(lcm_static STATIC IMPORTED)
  #SET_TARGET_PROPERTIES(lcm_static PROPERTIES 
  #IMPORTED_LOCATION /usr/local/angstrom/arm/arm-angstrom-linux-gnuea/lib/liblcm.la)
  
  target_link_libraries(hellofootbot 
    argos2_real_robot_swarmanoid_footbot
    argos2_real_robot_swarmanoid_common
    argos2_common_control_interface 
    argos2_common_utility
    lcm
    # any other library
  )

else( )
 # This does not work in simulation!

endif( )

  

