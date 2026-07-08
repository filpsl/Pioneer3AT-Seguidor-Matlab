# ros-pioneer3at (ROS 2 Humble + Gazebo Harmonic)

Modelo e simulação do robô **Pioneer 3-AT** para **ROS 2 Humble** e
**Gazebo Harmonic (gz-sim 8)**.

Este pacote é uma **migração** do projeto original de Dereck Wonnacott, que era
para ROS 1 + Gazebo Classic. Toda a parte antiga (nós C++ de ponte, launch files
XML) foi arquivada em [`legacy_ros1/`](legacy_ros1/) para referência.

O que mudou na migração, em resumo:

| Antes (ROS 1 / Gazebo Classic) | Agora (ROS 2 Humble / Gazebo Harmonic) |
|---|---|
| URDF só visual (não carregava no Gazebo) | URDF/xacro com colisão + inércia + plugins |
| 3 nós C++ sobre `gazebo::transport` | Plugins do gz-sim + `ros_gz_bridge` (sem C++) |
| `catkin` / `package.xml` formato 1 | `ament_cmake` / `package.xml` formato 3 |
| launch `.launch` (XML) | launch `.launch.py` (Python) |
| robô vinha do banco de modelos do Gazebo | modelo autossuficiente no próprio pacote |

---

## 1. Requisitos

- Ubuntu 22.04 (ou base equivalente, ex. Zorin OS 17)
- ROS 2 Humble (`ros-humble-desktop`)
- Gazebo Harmonic (`gz-sim` 8.x)

Instale os pacotes de integração:

```bash
sudo apt update
sudo apt install \
  ros-humble-ros-gz-sim \
  ros-humble-ros-gz-bridge \
  ros-humble-xacro \
  ros-humble-robot-state-publisher \
  ros-humble-joint-state-publisher-gui \
  ros-humble-teleop-twist-keyboard \
  ros-humble-rviz2
```

Confira as versões:

```bash
source /opt/ros/humble/setup.bash
gz sim --version      # deve ser 8.x (Harmonic)
```

## 2. Build

Coloque (ou crie um symlink d)este pacote dentro de um workspace colcon:

```bash
mkdir -p ~/ros2_ws/src
ln -s /caminho/para/ros-pioneer3at ~/ros2_ws/src/pioneer3at

cd ~/ros2_ws
colcon build --packages-select pioneer3at
source install/setup.bash
```

## 3. Rodar a simulação

Tudo em um comando (Gazebo + robô + ponte ROS↔Gazebo):

```bash
ros2 launch pioneer3at simulation.launch.py
```

Com o RViz2 (modelo + TF + LiDAR):

```bash
ros2 launch pioneer3at simulation.launch.py rviz:=true
```

Argumentos úteis:

| Argumento | Padrão | Descrição |
|---|---|---|
| `world` | `worlds/pioneer_world.sdf` | mundo `.sdf` a carregar |
| `rviz` | `false` | abre o RViz2 |
| `use_sim_time` | `true` | usa o relógio da simulação |

## 4. Dirigir o robô (teleop por teclado)

Em outro terminal (com o workspace "sourçado"):

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

Use as teclas indicadas na tela (`i`/`,` frente/ré, `j`/`l` girar). Os comandos
saem em `/cmd_vel`, são levados ao Gazebo pela ponte e movem o robô.

Teste rápido sem teclado:

```bash
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0.3}}'
```

## 5. Tópicos principais

| Tópico | Tipo | Sentido |
|---|---|---|
| `/cmd_vel` | `geometry_msgs/msg/Twist` | ROS → Gazebo (controle) |
| `/odom` | `nav_msgs/msg/Odometry` | Gazebo → ROS |
| `/tf` | `tf2_msgs/msg/TFMessage` | Gazebo → ROS (`odom`→`base_link`) |
| `/joint_states` | `sensor_msgs/msg/JointState` | Gazebo → ROS (rodas) |
| `/scan` | `sensor_msgs/msg/LaserScan` | Gazebo → ROS (LiDAR) |
| `/clock` | `rosgraph_msgs/msg/Clock` | Gazebo → ROS |

## 6. Visualizar só o modelo (sem Gazebo)

Para conferir a montagem do URDF com sliders de junta:

```bash
ros2 launch pioneer3at display.launch.py
```

## 7. Estrutura do pacote

```
urdf/pioneer3at.urdf.xacro   # descrição do robô (visual + colisão + inércia + plugins gz)
worlds/pioneer_world.sdf     # mundo do Gazebo Harmonic (chão, luz, obstáculo)
config/ros_gz_bridge.yaml    # mapeamento de tópicos ROS <-> Gazebo
config/pioneer3at.rviz       # configuração do RViz2
launch/
  simulation.launch.py       # bringup completo (gazebo + spawn + bridge + rviz)
  gazebo.launch.py           # só o Gazebo + mundo
  spawn_robot.launch.py      # robot_state_publisher + spawn no gz
  bridge.launch.py           # só a ponte ros_gz_bridge
  display.launch.py          # só RViz2 com o modelo (sem Gazebo)
meshes/                      # malhas STL (reaproveitadas do projeto original)
legacy_ros1/                 # código ROS 1 / Gazebo Classic original (referência)
```

## 8. Trabalho futuro

- Navegação autônoma com **Nav2** e SLAM com **slam_toolbox** (substituem o antigo
  `move_base` / `gmapping` / `amcl`). A base de simulação aqui já publica `/odom`,
  `/tf` e `/scan`, que são os pré-requisitos para isso.

## Créditos

- Projeto original ROS 1: **Dereck Wonnacott** (https://github.com/dawonn/ros-Pioneer3AT)
- Migração ROS 2 Humble + Gazebo Harmonic: **Filipe Ramos**
