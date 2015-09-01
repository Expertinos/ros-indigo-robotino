FILE(REMOVE_RECURSE
  "../src/robotino_local_move/msg"
  "../src/robotino_local_move/srv"
  "../msg_gen"
  "../srv_gen"
  "../msg_gen"
  "../srv_gen"
  "CMakeFiles/ROSBUILD_genmsg_cpp"
  "../msg_gen/cpp/include/robotino_local_move/LocalMoveAction.h"
  "../msg_gen/cpp/include/robotino_local_move/LocalMoveGoal.h"
  "../msg_gen/cpp/include/robotino_local_move/LocalMoveActionGoal.h"
  "../msg_gen/cpp/include/robotino_local_move/LocalMoveResult.h"
  "../msg_gen/cpp/include/robotino_local_move/LocalMoveActionResult.h"
  "../msg_gen/cpp/include/robotino_local_move/LocalMoveFeedback.h"
  "../msg_gen/cpp/include/robotino_local_move/LocalMoveActionFeedback.h"
  "../msg_gen/cpp/include/robotino_local_move/PathDisplacements.h"
  "../msg_gen/cpp/include/robotino_local_move/PathStatus.h"
  "../msg/LocalMoveAction.msg"
  "../msg/LocalMoveGoal.msg"
  "../msg/LocalMoveActionGoal.msg"
  "../msg/LocalMoveResult.msg"
  "../msg/LocalMoveActionResult.msg"
  "../msg/LocalMoveFeedback.msg"
  "../msg/LocalMoveActionFeedback.msg"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_genmsg_cpp.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
