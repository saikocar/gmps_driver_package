# 使い方: include(cmake/dw_version.cmake) の後、実行ファイルごとに dw_version_attach(<target>)。
# ソースでは #include "dw_version.hpp" して DW_VERSION(文字列)を使う。
set(DW_VERSION_DIR "${CMAKE_CURRENT_BINARY_DIR}/dw_version")
file(MAKE_DIRECTORY "${DW_VERSION_DIR}")
add_custom_target(${PROJECT_NAME}_dw_version_gen ALL
  COMMAND ${CMAKE_COMMAND} -DSRC_DIR=${CMAKE_CURRENT_SOURCE_DIR} -DOUT=${DW_VERSION_DIR}/dw_version.hpp
          -P ${CMAKE_CURRENT_SOURCE_DIR}/cmake/dw_version_gen.cmake
  BYPRODUCTS ${DW_VERSION_DIR}/dw_version.hpp
  COMMENT "dw_version: embedding git version")
function(dw_version_attach target)
  add_dependencies(${target} ${PROJECT_NAME}_dw_version_gen)
  target_include_directories(${target} PRIVATE ${DW_VERSION_DIR})
endfunction()
