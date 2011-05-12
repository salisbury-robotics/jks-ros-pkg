ADD_EXECUTABLE(chrome
    src/chrome.cpp
)

TARGET_LINK_LIBRARIES(chrome chai3d_complete)

# This is a BAD HACK for MSVC projects to put their targets in the right place!
IF(MSVC OR APPLE)
    SET_TARGET_PROPERTIES(chrome PROPERTIES PREFIX "../")
ENDIF(MSVC OR APPLE)
