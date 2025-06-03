#! /bin/bash

sudo cpupower frequency-set -u 2.0GHz # Limitation de la fréquence max des CPUs (pour éviter de faire surchauffer et ventiler mon PC)
xhost +local:multirobots # Nécessaire pour permettre l'utilisation d'interfaces graphiques dans le conteneur docker

# Lancement d'un conteneur docker, basé sur l\'image cristal-container
# -v utiliser pour monter des dossiers du host, pour pouvoir utiliser et modifier leur contenu dans le conteneur
docker run -it --rm --user="multirobots" --env="DISPLAY" \
--env="QT_X11_NO_MITSHM=1" \
--volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
--net host \
-v $HOME/multirobots_sim/multirobots_ws:/home/multirobots/multirobots_ws \
cristal-container 
