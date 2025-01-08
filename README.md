# LARM
Projet robot en UV LARM

## Challenge 1

Installation nécessaire de pyrealsense2
```
$ pip install pyrealsense2
```

### Déplacement
Le robot est capable de se déplacer dans tout l'espace, sans se bloquer et sans n'avoir besoin de s'arrêter.
Ses mouvements sont fluides et il anticipe la venue d'obstacles.
Il se déplace intelligemment dans l'espace afin d'explorer l'ensemble de la zone.

### Vision 
La caméra permet au robot de détecter la présence d'objets verts.
Si la forme détectée est suffisament imposante, il considère que l'objet est trouvé et envoie un message String dans un topic dédié.

### Fonctionalités supplémentaires 
Le robot possède plusieurs fonctions d'arrêt avec les différents éléments qu'il possède : le bumper avant ainsi que les roules qui se relâchent.
Les boutons du robot permettent de relancer le mouvement automatique.