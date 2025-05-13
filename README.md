<h1 align="center">🌹🌹🌹 WARDE 🌹🌹🌹</h1>
<h1 align="center">Wandering Autonomous Robot Donde Estas</h1>

### The Great MCE550 Project
`Warde` is a fully-integrated ROS 2-based behavior tree demo that autonomously roams, finds beer cans, picks them up, and places them into a drop-box. It leverages Gazebo simulation, BehaviorTree.CPP, ROS 2 actions/services, and a simple tagging mechanism to avoid revisiting the same target.

---

## Quick Start

### 1. Prerequisites

- Ubuntu 22.04 LTS  
- ROS 2 Humble Hawksbill (desktop)  
- Docker & Docker Compose (optional, for containerized runs)  
- `colcon` build tools  

### 2. ROS 2 Installation

```bash
# Setup locale
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8

# Add ROS 2 repo and keys
sudo apt update && sudo apt install -y curl gnupg2 lsb-release
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc   | sudo apt-key add -
sudo sh -c 'echo "deb http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main"   > /etc/apt/sources.list.d/ros2.list'
sudo apt update

# Install ROS 2 Humble (Desktop)
sudo apt install -y ros-humble-desktop

# Install build tools and dependencies
sudo apt install -y python3-colcon-common-extensions python3-rosdep

# Initialize rosdep
sudo rosdep init
rosdep update
```

Load ROS 2 environment in your `~/.bashrc`:

```bash
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

### 3. Clone & Build the Workspace

```bash
cd ~/Desktop/MCE550/warde
# Ensure 'src/' contains: robot_description, spawn_tools, warde_bt, etc.
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

---

## Launching the System

We provide a single launch file to start Gazebo + RViz, the nav server, both spawners, and the behavior tree:

```bash
ros2 launch warde_bt launch_all.launch.py   world:=$(ros2 pkg prefix robot_description)/models/myWorld/boxes_world.sdf   use_sim_time:=true   rviz_config_file:=$(ros2 pkg prefix robot_description)/rviz/sim.config.rviz
```

Behind the scenes, this does:

1. `ros2 launch robot_description launch_sim.launch.py ...`  
2. `ros2 run robot_nav navigate_node`  
3. `ros2 run spawn_tools spawn_box_node`  
4. `ros2 run spawn_tools spawn_beer_node`  
5. `ros2 run warde_bt warde_bt_main`

---

## Features & Workflow

1. **Wandering** mode: random exploration when no beer is found or after placement.  
2. **Beer detection**: TF-listener scans `/tf` for frames named `beer*`.  
3. **Closest-beer selection**: picks the nearest beer frame.  
4. **Tagged beers**: once collected, beer frames get “tagged” and ignored on future loops.  
5. **Precise approach**: robot stops 1 m from beer or from the front face of the box.  
6. **Behavior Tree**: mission sequence built with BehaviorTree.CPP:  
   - `ConditionBoxPresent`, `ConditionBeerPresent`  
   - `ActionGetClosestBeer`, `ActionNavigate`, `ActionManipulate`  
   - Fallback to wandering on any failure, loop indefinitely.  

---

## Containerized Run (Docker)

1. Build the Docker image:

    ```bash
    docker build -t warde_ws:humble .
    ```

2. Run it (with X11 forwarding & host networking):

    ```bash
    xhost +local:root
    docker run -it --rm       --net=host       -e DISPLAY=$DISPLAY       -v /tmp/.X11-unix:/tmp/.X11-unix       warde_ws:humble
    ```

3. Alternatively, build and start all services with Docker Compose:

    ```bash
    xhost +local:root && docker-compose up --build
    ```

---

## Accessing Services

You can call any ROS 2 service running in the container directly from your host shell or by opening a shell inside the container.

### From the Host Shell

Make sure you have sourced the workspace overlay on your host so ROS 2 recognizes your custom packages:

```bash
cd ~/Desktop/MCE550/warde
source install/setup.bash
ros2 service list
ros2 service call /spawn_beer spawn_tools/srv/SpawnBeer   "{entity_name: 'beer2', x: 2.0, y: 3.0, z: 0.25}"
```

### From Inside the Container

```bash
docker exec -it warde_ws bash
source /ros2_ws/install/setup.bash
ros2 service list
ros2 service call /spawn_box spawn_tools/srv/SpawnBeer   "{entity_name: 'box1', x: 0.0, y: 0.0, z: 0.0}"
```

---

## Best Practices

- Keep ROS 2 packages minimal and declare all `<depend>` tags  
- Use `--symlink-install` for faster dev cycles  
- Lock Gazebo/ROS versions in Docker for reproducibility  
- Write small, single-purpose Behavior Tree nodes  
- Handle service timeouts and failures gracefully  
- Version-control all your launch and config files  

---

## Further Reading

- [BehaviorTree.CPP Tutorial](https://www.behaviortree.dev/)  
- [ROS 2 Navigation 2](https://navigation.ros.org/)  
- [Gazebo SDF Documentation](http://sdformat.org/)  

---

*Enjoy building and wandering with WARDE!*