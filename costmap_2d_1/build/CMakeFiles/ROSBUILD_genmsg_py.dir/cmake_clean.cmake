FILE(REMOVE_RECURSE
  "../msg_gen"
  "../src/costmap_2d_1/msg"
  "../msg_gen"
  "CMakeFiles/ROSBUILD_genmsg_py"
  "../src/costmap_2d_1/msg/__init__.py"
  "../src/costmap_2d_1/msg/_VoxelGrid.py"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_genmsg_py.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
