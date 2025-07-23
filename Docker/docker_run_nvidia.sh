#! /bin/bash

# You can uncomment some of the commands below if your computer has perfomance issues
sudo cpupower frequency-set -u 2.0GHz # Limits max frequency of CPUs (to avoid the overheating and noisy ventilation of the computer)
#sudo chmod 600 /swapfile
#sudo mkswap /swapfile
#sudo swapon /swapfile

xhost +local:multirobots # Nécessaire pour permettre l'utilisation d'interfaces graphiques dans le conteneur docker

# Lancement d'un conteneur docker, basé sur l\'image cristal-container
# -v utiliser pour monter des dossiers du host, pour pouvoir utiliser et modifier leur contenu dans le conteneur
docker run --security-opt seccomp=unconfined --runtime=nvidia --gpus all -e NVIDIA_DRIVER_CAPABILITIES=all -it --rm --user="multirobots" --env="DISPLAY" \
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
