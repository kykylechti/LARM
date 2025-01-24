# LARM
Projet robot en UV LARM

## Dépendances

Installation nécessaire de pyrealsense2
```sh
$ pip install pyrealsense2
```
Installation nécessaire de cv2
```sh
$ pip install opencv-python
```
Installation des ROS interfaces pour permettre de communiquer avec les boutons, bumpers...
```sh
cd $ROS_WORKSPACE
git clone https://github.com/imt-mobisyst/pkg-interfaces.git
colcon build --base-path pkg-interfaces
source ./install/setup.bash
```

## Install
```sh
git clone https://github.com/kykylechti/uvlarm-wall-e
colcon build --base-path pkg-interfaces
source ./install/setup.bash
```

## Get Started
Robot mobile
```sh
ros2 launch grp_pibot26 tbot_v1_launch.yaml
```

Simulation
```sh
ros2 launch grp_pibot26 simulation_v1_launch.yaml
```

### Déplacement

uvlarm-wall-e/grp_pibot26/scripts/direct_robot

Le robot est capable de se déplacer dans tout l'espace, sans se bloquer et sans n'avoir besoin de s'arrêter.
Ses mouvements sont fluides et il anticipe la venue d'obstacles.
Il se déplace aléatoirement dans l'espace afin d'explorer l'ensemble de la zone.

### Vision

uvlarm-wall-e/grp_pibot26/scripts/vision

Cette node repert la présence d'objets verts dans le champ de la caméra. Chaque objet détecté est comparé à un template du fantome. Si la corrélation est suffisante, un son est produit par le robot, un message est envoyé dans un topic dédié et la distance au robot est calculé.
Les objets détectés sont placés sur la carte en rouge pour les objets verts quelconques et en vert pour les fantomes. Deux points ne peuvent pas être placés à proximité et chaque fantome est identifié avec une ID unique.

### Fonctionalités supplémentaires 
Le robot possède plusieurs fonctions d'arrêt avec les différents éléments qu'il possède : le bumper avant ainsi que les roules qui se relâchent.
Les boutons du robot permettent de relancer le mouvement automatique.

### Types de message personnalisés
Création de message personnalisés pour certains topics. Ces messages permettent par exemple de créer des flux d'image avec des coordonnées associées sur cette image. 

### En cours de développement
Mouvement intelligent basé sur un algorithme de recherche du plus court chemin. Une fois ce chemin trouvé, l'objectif est d'identifier les points où des changements de direction ont lieu, puis d'orienter le robot et de l'envoyer jusqu'au prochain point.
Pour cela, nous cherchons les zones non identifiées sur la carte (-1) et sélectionnons les zones potentiellement accessibles pour le robot.