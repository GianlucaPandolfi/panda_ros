cmake_minimum_required(VERSION 3.8)
project(onnxruntime)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY models/ DESTINATION models)
install(DIRECTORY lib/ DESTINATION lib)

ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/env-hooks/onnxruntime_library_path.sh")

ament_package()
