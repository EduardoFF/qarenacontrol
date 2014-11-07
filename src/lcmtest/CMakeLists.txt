add_executable(fakepublisher fakepublisher.cc)
add_executable(fakelistener fakelistener.cc)
target_link_libraries(fakelistener ${LCM_LIBRARY})
target_link_libraries(fakepublisher ${LCM_LIBRARY})
