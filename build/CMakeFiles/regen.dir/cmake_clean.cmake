FILE(REMOVE_RECURSE
  "CMakeFiles/regen"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/regen.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
