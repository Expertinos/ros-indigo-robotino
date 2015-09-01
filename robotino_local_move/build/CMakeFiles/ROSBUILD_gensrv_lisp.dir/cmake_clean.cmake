FILE(REMOVE_RECURSE
  "../src/robotino_local_move/msg"
  "../src/robotino_local_move/srv"
  "../msg_gen"
  "../srv_gen"
  "../msg_gen"
  "../srv_gen"
  "CMakeFiles/ROSBUILD_gensrv_lisp"
  "../srv_gen/lisp/FullPath.lisp"
  "../srv_gen/lisp/_package.lisp"
  "../srv_gen/lisp/_package_FullPath.lisp"
  "../srv_gen/lisp/Stop.lisp"
  "../srv_gen/lisp/_package.lisp"
  "../srv_gen/lisp/_package_Stop.lisp"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_gensrv_lisp.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
