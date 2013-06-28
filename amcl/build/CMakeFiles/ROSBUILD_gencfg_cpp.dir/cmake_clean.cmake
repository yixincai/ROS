FILE(REMOVE_RECURSE
  "CMakeFiles/ROSBUILD_gencfg_cpp"
  "../cfg/cpp/amcl/AMCLConfig.h"
  "../docs/AMCLConfig.dox"
  "../docs/AMCLConfig-usage.dox"
  "../src/amcl/cfg/AMCLConfig.py"
  "../docs/AMCLConfig.wikidoc"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_gencfg_cpp.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
