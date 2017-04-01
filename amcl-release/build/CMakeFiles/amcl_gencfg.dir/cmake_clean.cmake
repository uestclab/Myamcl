FILE(REMOVE_RECURSE
  "CMakeFiles/amcl_gencfg"
  "devel/include/amcl/AMCLConfig.h"
  "devel/share/amcl/docs/AMCLConfig.dox"
  "devel/share/amcl/docs/AMCLConfig-usage.dox"
  "devel/lib/python2.7/dist-packages/amcl/cfg/AMCLConfig.py"
  "devel/share/amcl/docs/AMCLConfig.wikidoc"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/amcl_gencfg.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
