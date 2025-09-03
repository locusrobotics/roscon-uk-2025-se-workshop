# ROSCon UK Workshop on State Estimation in ROS 2

## Introduction

The State Estimation in ROS 2 workshop is being presented as part of ROSCon UK 2025, which is scheduled to be held in Edinburgh, UK from 15 - 17 September 2025. This repository contains supporting materials for the workshop, as well as setup instructions for various platforms.

## Setup

We have created a Docker image for the workshop, and attendees are encouraged to use it for simplicity. However, the wide availability of the packages we'll be using for the workshop, along with their relatively light dependencies, mean that it should work equally well if cloned and run in a native ROS 2 workspace.

### Docker installation
If you do not have Docker installed, please follow the instructions [here](https://docs.docker.com/engine/install/). You will also need the `docker-compose` plugin.

MacOS users should also install [XQuartz](https://www.xquartz.org), as the container will need to display `rviz2`.

### Docker image installation

The most straightforward way to start the container is the following:

```
wget <URL for docker-compose.yaml>
docker compose run <name of container>
```

#### Docker image shortcuts

For convenience, we have provided a number of handy shortcuts for working with the workspace in the Docker container.

##### Directory shortcuts

`$ws` - Variable for the workspace root  
`$bags` - Variable for the bag directory  
`$taskN` - Variable for the directory for task N (where N ranges from 1 to 8)

##### Command aliases

`s` - Sources the workspace `ws/install/setup.bash`  
`cb` - Buulds the workspace by running `cd /root/ws && catkin build --symlink-install && cd -`

### Building from source

If you are unable or unwilling to use Docker, you have the option to clone the repository and build it locally:

```
sudo apt install git git-lfs
source /opt/ros/<ROS DISTRO>/setup.bash
mkdir -p ws/src
cd ws/src
git clone https://github.com/ayrton04/roscon-uk-2025-se-workshop.git
cd ..
colcon build --symlink-install
```

Then, in any terminals you open, you can run

```
source ws/install/setup.bash
```

You should also decompress the bag files as described in the next section.

Everything should then work in the same way as it would within the Docker container.

## ROS Bags

This workshop makes use of a number of ROS 2 bag files. The repository contains compressed versions of them, along with a small bash script for decompressing them. This has already been executed for you in the Docker container, but users who have manually cloned the GitHub repository should run

```
cd ws/roscon-uk-2025-se-workshop/bags
./decompress.sh
```

## Tasks

### Session 1: `robot_localization`

#### [Task 1 - Basic Planar Robot](task1/README.md)

#### [Task 2 - Advanced Parameters](task2/README.md)

#### [Task 3 - "Two Tier" Setup](task3/README.md)

#### [Task 4 - GPS Data](task4/README.md)

#### [Task 5 - Operating in 3D](task5/README.md)

### Session 2: `fuse`

#### [Task 6 - Back to the Planar](task6/README.md)

#### [Task 7 - Déjà Two (-tier Setup)](task7/README.md)

#### [Task 8 - Writing a `fuse` Plugin](task8/README.md)

