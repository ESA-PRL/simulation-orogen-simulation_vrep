FILE(REMOVE_RECURSE
  "CMakeFiles/check-uptodate"
  "../.orogen/simulation_vrep.orogen"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/check-uptodate.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
