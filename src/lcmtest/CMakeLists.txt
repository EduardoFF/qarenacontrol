add_executable(fakepublisher fakepublisher.cc)
add_executable(fakelistener fakelistener.cc)
target_link_libraries(fakelistener lcm)
target_link_libraries(fakepublisher lcm)
