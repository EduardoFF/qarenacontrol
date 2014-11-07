add_library(footbot_central SHARED
  includes/matlab/mengine.cpp
  includes/lcm/lcmhandler.cpp
  includes/lcm/lcmthread.cpp
  includes/thread/threadengine.cpp
  includes/beans/node.cpp
  footbot_central.cpp
)
target_link_libraries(footbot_central ${MATLABLIB_LIBRARY} lcm argos2_simulator_dynamic_linking)
