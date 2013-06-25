FILE(REMOVE_RECURSE
  "../msg_gen"
  "../src/costmap_2d/msg"
  "../msg_gen"
  "CMakeFiles/ROSBUILD_genmsg_py"
  "../src/costmap_2d/msg/__init__.py"
  "../src/costmap_2d/msg/_VoxelGrid.py"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_genmsg_py.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
