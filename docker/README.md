# Developing inside a docker container

In this tutorial, we explain how to develop, build and run this project inside a docker container.

The container user and password are:

**developer:developer**

## Prerequisites

First, you must install `docker`.

```bash
curl -fsSL https://get.docker.com -o get-docker.sh
sudo sh get-docker.sh
```

## Build and start up a container

From the root of the repo, run this script to create an image and a container:

```bash
./docker/dock.sh [container-name] build
```

Run this to start the container with an interactive shell:

```bash
./docker/dock.sh [container-name] start
```

Run this to stop the container:

```bash
./docker/dock.sh [container-name] stop
```

Finally, run this to remove container and image:

```bash
./docker/dock.sh [container-name] clean
```

## Working with the code

Inside the container, the repo is bind-mounted at `~/ws`, so every package under
`~/ws/src` is a normal colcon package. Build and run it with:

```bash
cd ~/ws
./bin/update.sh    # only once: rosdep install
./bin/build.sh
source install/setup.bash
ros2 launch commander_server commander.launch.py
```

Run the tests with:

```bash
./bin/test.sh
```
