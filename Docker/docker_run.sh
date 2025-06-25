#! /bin/bash

sudo cpupower frequency-set -u 2.0GHz # Limitation de la fréquence max des CPUs (pour éviter de faire surchauffer et ventiler mon PC)
sudo chmod 600 /swapfile
sudo mkswap /swapfile
sudo swapon /swapfile
xhost +local:multirobots # Nécessaire pour permettre l'utilisation d'interfaces graphiques dans le conteneur docker

# Lancement d'un conteneur docker, basé sur l\'image cristal-container
# -v utiliser pour monter des dossiers du host, pour pouvoir utiliser et modifier leur contenu dans le conteneur
docker run --security-opt seccomp=unconfined -it --rm --user="multirobots" --env="DISPLAY" \
--env="QT_X11_NO_MITSHM=1" \
--volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
--net host \
--device /dev/dri \
-v /dev/dri:/dev/dri \
-v $HOME/multirobots_sim/multirobots_ws:/home/multirobots/multirobots_ws \
-v $HOME/multirobots_sim/.git:/home/multirobots/.git \
-v $HOME/multirobots_sim/.gitmodules:/home/multirobots/.gitmodules \
-v $HOME/multirobots_sim/.gitattributes:/home/multirobots/.gitattributes \
-v $HOME/multirobots_sim/.gitignore:/home/multirobots/.gitignore \
-v $HOME/multirobots_sim/README.md:/home/multirobots/README.md \
cristal-container 
