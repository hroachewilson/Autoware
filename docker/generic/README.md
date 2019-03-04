# Autoware Docker
To use the Autoware Docker, first make sure the NVIDIA drivers, Docker and nvidia-docker are properly installed.

[Docker installation](https://docs.docker.com/engine/installation/linux/docker-ce/ubuntu/)


[nvidia-docker installation](https://github.com/NVIDIA/nvidia-docker)

## How to Build
```
$ cd Autoware/docker/generic/

# Ubuntu 16.04 (Kinetic)
$ sh build.sh kinetic

# Ubuntu 16.04 (Kinetic with OpenGL for RVIZ)
$ sh build.sh kinetic nvidia

```

## How to Run
```

# Ubuntu 16.04 (Dockerhub Kinetic)
$ ./run.sh -t latest-kinetic

# Ubuntu 16.04 (AEV Kinetic)
$ ./run-built.sh autoware-kinetic

# Ubuntu 16.04 (AEV Kinetic with OpenGL for RVIZ)
$ ./run-built.sh autoware-kinetic-nvidia

```

|Option|Default|Description|
|---|---|---|
|-h||Show `Usage: $0 [-t <tag>] [-r <repo>] [-s <Shared directory>]`|
|-t|latest-kinetic|Specify tag|
|-r|autoware/autoware|Specify repo|
|-s|/home/$USER/shared_dir|Specify shared dir|
