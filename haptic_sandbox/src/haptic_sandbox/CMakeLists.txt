include_directories(.)

rosbuild_add_library(haptic_sandbox
#src/geometry/GenericSampler.cpp
#src/geometry/PointSetSampler.cpp
#src/geometry/VoxelSampler.cpp
#src/geometry/RigidBody.cpp
#src/ToolBody.cpp
#marker_interaction_tool.cpp
haptic_interaction_tool.cpp
#manifest.xml
)
