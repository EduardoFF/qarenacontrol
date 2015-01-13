add_library(footbot_an SHARED
  footbot_an.cpp
  includes/thread/threadengine.cpp
  includes/lcm/lcmthread.cpp
  includes/lcm/lcmhandler.cpp
  includes/beans/node.cpp
  includes/lcm/protectedmutex.cpp
  ${orca_IMPLEMENTATIONS}
  ${hrvo_IMPLEMENTATIONS}
  ${agentvo_IMPLEMENTATIONS}
)
target_link_libraries(footbot_an lcm argos2_simulator_dynamic_linking)
