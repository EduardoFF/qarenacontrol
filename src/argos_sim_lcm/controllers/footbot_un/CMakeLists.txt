add_library(footbot_un SHARED
  footbot_un.cpp
  includes/thread/threadengine.cpp
  includes/lcm/lcmthread.cpp
  includes/lcm/lcmhandler.cpp
  includes/beans/node.cpp
  includes/lcm/protectedmutex.cpp
  ${orca_IMPLEMENTATIONS}
  ${hrvo_IMPLEMENTATIONS}
  ${agentvo_IMPLEMENTATIONS}
)
target_link_libraries(footbot_un lcm argos2_simulator_dynamic_linking)
