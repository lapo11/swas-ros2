cmake_minimum_required(VERSION 3.8)
project(swas_hmi)
find_package(ament_cmake REQUIRED)

# npm stays an explicit frontend build step; colcon never downloads npm packages.
if(NOT EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/dist/index.html")
  message(FATAL_ERROR "Frontend dist/index.html is missing. Run npm install && npm run build inside swas_hmi, then build this ROS package.")
endif()

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
  PATTERN "__pycache__" EXCLUDE
)
install(DIRECTORY dist/
  DESTINATION share/${PROJECT_NAME}/web
  PATTERN "COLCON_IGNORE" EXCLUDE
)
ament_package()
