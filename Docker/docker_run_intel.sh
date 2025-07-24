#! /bin/bash

# You can uncomment some of the commands below if your computer has perfomance issues
#sudo cpupower frequency-set -u 2.0GHz # Limits max frequency of CPUs (to avoid the overheating and noisy ventilation of the computer)

xhost +local:multirobots # Needed to allow acces to the X11 server and use graphical user interfaces in Docker (for Gazebo GUI, terminator, ...)

# Run the docker container
# -v is used to mount the folders from host, so that you can use & modify them in the container
docker run --security-opt seccomp=unconfined -it --rm --user="multirobots" --env="DISPLAY" \
--env="QT_X11_NO_MITSHM=1" \
--volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
--net host \
--device /dev/dri \
-v /dev/dri:/dev/dri \
-v $PWD/multirobots_ws:/home/multirobots/multirobots_ws \
-v $PWD/.git:/home/multirobots/.git \
-v $PWD/.gitmodules:/home/multirobots/.gitmodules \
-v $PWD/.gitattributes:/home/multirobots/.gitattributes \
-v $PWD/.gitignore:/home/multirobots/.gitignore \
-v $PWD/README.md:/home/multirobots/README.md \
cristal-container 
