include_directories(.)

rosbuild_add_library(haptic_sandbox
#src/geometry/GenericSampler.cpp
#src/geometry/PointSetSampler.cpp
#src/geometry/VoxelSampler.cpp
#src/geometry/RigidBody.cpp
#src/ToolBody.cpp
#marker_interaction_tool.cpp


abstract_handle.cpp
tf_scenegraph_object.cpp
camera_node.cpp
manipulator_node.cpp
user_entity.cpp
abstract_interaction_tool.cpp
haptic_interaction_tool.cpp

#tool_entity.cpp
#abstract_coupling.cpp
#manifest.xml
)
