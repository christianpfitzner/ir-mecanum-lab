cmake_minimum_required(VERSION 3.8)
project(mecanum_lab_interfaces)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Ein Service, keine Messages: Roboter spaeter zur laufenden Simulation hinzufuegen.
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/SpawnRobot.srv"
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
