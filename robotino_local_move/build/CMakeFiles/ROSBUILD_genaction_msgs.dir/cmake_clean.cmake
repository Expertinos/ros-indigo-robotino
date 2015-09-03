FILE(REMOVE_RECURSE
  "../src/robotino_local_move/msg"
  "../src/robotino_local_move/srv"
  "../msg_gen"
  "../srv_gen"
  "../msg_gen"
  "../srv_gen"
  "CMakeFiles/ROSBUILD_genaction_msgs"
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
  INCLUDE(CMakeFiles/ROSBUILD_genaction_msgs.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
