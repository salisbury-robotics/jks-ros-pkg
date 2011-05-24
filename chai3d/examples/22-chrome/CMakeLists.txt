ADD_EXECUTABLE(22-chrome
    src/22-chrome.cpp
)

TARGET_LINK_LIBRARIES(22-chrome chai3d)

# This is a BAD HACK for MSVC projects to put their targets in the right place!
IF(MSVC OR APPLE)
    SET_TARGET_PROPERTIES(chrome PROPERTIES PREFIX "../")
ENDIF(MSVC OR APPLE)
