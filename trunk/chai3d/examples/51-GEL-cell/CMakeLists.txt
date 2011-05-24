# GEL projects require sources in external/GEL
SET(GEL_DIRECTORY "${CHAI3D_SOURCE_DIR}/modules/GEL")
INCLUDE_DIRECTORIES(${GEL_DIRECTORY})

ADD_EXECUTABLE(51-GEL-cell
    src/51-GEL-cell.cpp
    ${GEL_DIRECTORY}/CGELSkeletonLink.cpp
    ${GEL_DIRECTORY}/CGELMesh.cpp
    ${GEL_DIRECTORY}/CGELSkeletonNode.cpp
    ${GEL_DIRECTORY}/CGELWorld.cpp
)

TARGET_LINK_LIBRARIES(51-GEL-cell chai3d)

# This is a BAD HACK for MSVC projects to put their targets in the right place!
IF(MSVC OR APPLE)
    SET_TARGET_PROPERTIES(GEL-cell PROPERTIES PREFIX "../")
ENDIF(MSVC OR APPLE)
