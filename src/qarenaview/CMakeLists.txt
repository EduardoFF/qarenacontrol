
#find_package(Boost COMPONENTS system filesystem serialization graph iostreams REQUIRED)

FIND_PACKAGE(Qt4 REQUIRED)
INCLUDE(${QT_USE_FILE})
ADD_DEFINITIONS(${QT_DEFINITIONS})

set( qarenaview_HEADERS
  edge.h
  node.h
  mouse.h
  graphwidget.h
  qlcmhandler.h
  robotpose.h
)

set( qarenaview_SRCS
  edge.cc
  node.cc
  mouse.cc
  graphwidget.cc
  qlcmhandler.cc
  robotpose.cc
)

QT4_WRAP_CPP(qarenaview_HEADERS_MOC ${qarenaview_HEADERS})

add_library(qarenaview ${qarenaview_HEADERS_MOC} ${qarenaview_SRCS})
add_executable(qarenaview-test main.cpp)
target_link_libraries(qarenaview-test qarenaview ${QT_LIBRARIES} ${LCM_LIBRARY})
#add_executable(testclient testclient.cpp)
#target_link_libraries(testclient trackerinterface)
