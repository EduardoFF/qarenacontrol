add_library(footbot_central SHARED
  mEngine.cpp
  footbot_central.cpp
  includes/thread/threadengine.cpp
  includes/lcm/lcmthread.cpp
  includes/lcm/lcmhandler.cpp
  includes/beans/node.cpp
)
target_link_libraries(footbot_central ${MATLABLIB_LIBRARY} lcm argos2_simulator_dynamic_linking)
