SHELL := /bin/bash
ROS_SETUP ?= /opt/ros/jazzy/setup.bash
IMAGE ?= nullspace_mpc:jazzy
LAUNCH_ARGS ?=
AGENDA ?= data/eval_demo/agenda.yaml
with_ros = source "$(ROS_SETUP)" && source install/local_setup.bash &&
launch = $(with_ros) ros2 launch nullspace_mpc_bringup
evaluate = $(with_ros) ros2 run nullspace_mpc_bringup run_evaluation.py

X11_ARGS = -e DISPLAY -e QT_X11_NO_MITSHM=1 -v /tmp/.X11-unix:/tmp/.X11-unix:ro
ifneq ($(wildcard $(XAUTHORITY)),)
X11_ARGS += -v "$(XAUTHORITY)":/tmp/nullspace-xauth:ro -e XAUTHORITY=/tmp/nullspace-xauth
endif
docker_run = docker run --rm -it --network host --ipc host $(X11_ARGS) \
  --user "$$(id -u):$$(id -g)" -v "$(CURDIR)":/ws -w /ws

.PHONY: build clean install_deps setup_docker_cpu setup_docker_gpu run_docker_cpu run_docker_gpu \
  record view_rosbag gazebo_world mapping gmapping navigation navigation_nullspace_mpc \
  navigation_nullspace_mpc_lite navigation_mppi navigation_mppi_unicycle eval_demo_nullspace_mpc eval_demo_mppi \
  eval_ten_small eval_ten_large

build:
	source "$(ROS_SETUP)" && colcon build --base-paths src --cmake-args -DCMAKE_BUILD_TYPE=Release

clean:
	rm -rf build install log

install_deps:
	source "$(ROS_SETUP)" && rosdep install -y --from-paths src --ignore-src --rosdistro jazzy

setup_docker_cpu setup_docker_gpu:
	docker build -t "$(IMAGE)" -f docker/Dockerfile .

run_docker_cpu:
	$(docker_run) "$(IMAGE)" bash

run_docker_gpu:
	$(docker_run) --gpus all -e NVIDIA_DRIVER_CAPABILITIES=all "$(IMAGE)" bash

record:
	mkdir -p rosbag && cd rosbag && source "$(ROS_SETUP)" && ros2 bag record -a

view_rosbag:
	$(launch) rosbag_play.launch.py $(LAUNCH_ARGS)

gazebo_world:
	$(launch) gazebo_world.launch.py $(LAUNCH_ARGS)

mapping:
	$(launch) mapping.launch.py $(LAUNCH_ARGS)

gmapping: mapping

navigation navigation_nullspace_mpc:
	$(launch) navigation.launch.py local_planner:=nullspace_mpc $(LAUNCH_ARGS)

navigation_nullspace_mpc_lite:
	$(with_ros) OMP_WAIT_POLICY=PASSIVE ros2 launch nullspace_mpc_bringup navigation.launch.py \
	  controller_mode:=lite show_gazebo_gui:=false gazebo_headless:=true $(LAUNCH_ARGS)

navigation_mppi:
	$(launch) navigation.launch.py local_planner:=mppi_h $(LAUNCH_ARGS)

navigation_mppi_unicycle:
	$(launch) navigation.launch.py local_planner:=mppi_unicycle $(LAUNCH_ARGS)

eval_demo_nullspace_mpc:
	$(evaluate) --agenda_yaml_path "$(AGENDA)" --controller nullspace_mpc

eval_demo_mppi:
	$(evaluate) --agenda_yaml_path "$(AGENDA)" --controller mppi_h

eval_ten_small:
	$(evaluate) --agenda_yaml_path data/eval_ten/small/agenda_small.yaml

eval_ten_large:
	$(evaluate) --agenda_yaml_path data/eval_ten/large/agenda_large.yaml
