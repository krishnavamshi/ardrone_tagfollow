FILE(REMOVE_RECURSE
  "CMakeFiles/ROSBUILD_gencfg_cpp"
  "../cfg/cpp/ardrone_tagfollow/dynamicConfig.h"
  "../docs/dynamicConfig.dox"
  "../docs/dynamicConfig-usage.dox"
  "../src/ardrone_tagfollow/cfg/dynamicConfig.py"
  "../docs/dynamicConfig.wikidoc"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_gencfg_cpp.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
