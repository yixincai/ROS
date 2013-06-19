FILE(REMOVE_RECURSE
  "CMakeFiles/ROSBUILD_gencfg_cpp"
  "../cfg/cpp/move_base_test/MoveBaseConfig.h"
  "../docs/MoveBaseConfig.dox"
  "../docs/MoveBaseConfig-usage.dox"
  "../src/move_base_test/cfg/MoveBaseConfig.py"
  "../docs/MoveBaseConfig.wikidoc"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_gencfg_cpp.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
