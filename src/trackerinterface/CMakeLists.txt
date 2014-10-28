
add_library(trackerinterface client.cpp)
add_executable(testclient testclient.cpp)
target_link_libraries(testclient trackerinterface)
