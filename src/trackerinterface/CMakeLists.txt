
find_package(Boost COMPONENTS system filesystem serialization graph iostreams REQUIRED)
add_executable(trackerinterface trackerinterface.cc)
#target_link_libraries(trackerinterface ${Boost_LIBRARIES})
target_link_libraries(trackerinterface ${Boost_LIBRARIES} ${LCM_LIBRARY})
#add_executable(testclient testclient.cpp)
#target_link_libraries(testclient trackerinterface)
