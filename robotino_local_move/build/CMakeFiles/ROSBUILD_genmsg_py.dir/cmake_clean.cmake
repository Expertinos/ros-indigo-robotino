FILE(REMOVE_RECURSE
  "../src/robotino_local_move/msg"
  "../src/robotino_local_move/srv"
  "../msg_gen"
  "../srv_gen"
  "../msg_gen"
  "../srv_gen"
  "CMakeFiles/ROSBUILD_genmsg_py"
  "../src/robotino_local_move/msg/__init__.py"
  "../src/robotino_local_move/msg/_LocalMoveAction.py"
  "../src/robotino_local_move/msg/_LocalMoveGoal.py"
  "../src/robotino_local_move/msg/_LocalMoveActionGoal.py"
  "../src/robotino_local_move/msg/_LocalMoveResult.py"
  "../src/robotino_local_move/msg/_LocalMoveActionResult.py"
  "../src/robotino_local_move/msg/_LocalMoveFeedback.py"
  "../src/robotino_local_move/msg/_LocalMoveActionFeedback.py"
  "../src/robotino_local_move/msg/_PathDisplacements.py"
  "../src/robotino_local_move/msg/_PathStatus.py"
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
  INCLUDE(CMakeFiles/ROSBUILD_genmsg_py.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
