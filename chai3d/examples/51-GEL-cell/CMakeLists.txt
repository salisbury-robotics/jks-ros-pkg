# GEM projects require sources in external/GEM
SET(GEM_DIRECTORY "${CHAI3D_SOURCE_DIR}/external/GEM")
INCLUDE_DIRECTORIES(${GEM_DIRECTORY})

ADD_EXECUTABLE(GEM-cell
    src/GEM_cell.cpp
    ${GEM_DIRECTORY}/CDefLink.cpp
    ${GEM_DIRECTORY}/CDefMesh.cpp
    ${GEM_DIRECTORY}/CDefNode.cpp
    ${GEM_DIRECTORY}/CDefWorld.cpp
)

TARGET_LINK_LIBRARIES(GEM-cell chai3d_complete)

# This is a BAD HACK for MSVC projects to put their targets in the right place!
IF(MSVC OR APPLE)
    SET_TARGET_PROPERTIES(GEM-cell PROPERTIES PREFIX "../")
ENDIF(MSVC OR APPLE)
