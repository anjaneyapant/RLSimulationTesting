cmake_minimum_required(VERSION 3.8)
project(my_gazebo_simulation)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)

# Install the `launch` directory
install(DIRECTORY launch/ DESTINATION share/${PROJECT_NAME}/launch)

# Install the `worlds` directory
install(DIRECTORY worlds/ DESTINATION share/${PROJECT_NAME}/worlds)

# Install any scripts in the `my_gazebo_scripts` directory
install(DIRECTORY my_gazebo_scripts/ DESTINATION lib/${PROJECT_NAME})

# Install the train_trpo_turtlebot.py as an executable
install(
  PROGRAMS
    my_gazebo_scripts/train_trpo_turtlebot.py
  DESTINATION lib/${PROJECT_NAME}
)

# Uncomment the following section if you are adding tests
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # Skip linter for copyright checks (adjust if necessary)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# Mark the package for ament
ament_package()
