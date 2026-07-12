cmake_minimum_required(VERSION 3.8)
project(safety_metrics)

find_package(ament_cmake REQUIRED)

# ヘッダオンリーパッケージ: ビルド対象のソース/ライブラリは無く、
# includeディレクトリをインストール・エクスポートするだけでよい。
add_library(${PROJECT_NAME} INTERFACE)
target_include_directories(${PROJECT_NAME} INTERFACE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

install(DIRECTORY include/
  DESTINATION include
)

install(TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
)

ament_export_targets(export_${PROJECT_NAME})
ament_export_include_directories(include)

ament_package()
