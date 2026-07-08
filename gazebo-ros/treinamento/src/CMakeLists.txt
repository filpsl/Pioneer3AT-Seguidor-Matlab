cmake_minimum_required(VERSION 3.8)
project(pioneer3at)

find_package(ament_cmake REQUIRED)

# Este pacote é apenas de recursos (descrição, mundos, launch, config, malhas).
# Não há mais código C++/Python de nós próprios: a integração com o Gazebo
# Harmonic é feita por plugins do gz-sim + ros_gz_bridge.
install(
  DIRECTORY
    urdf
    launch
    config
    meshes
    models
    worlds
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
