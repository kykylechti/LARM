launch:
- node: 
    pkg: "grp_pibot26"
    exec: "direct_robot"
    param: 
    - { name: "defaultSpeed", value: 0.7 }
    - { name: "defaultRotSpeed", value: 1.0 }
    - { name: "topicVel", value: "/multi/cmd_nav" }

- node:
    pkg: "grp_pibot26"
    exec: "vision"
    param: 
    - { name: "color", value: 55 }
    - { name: "rayon_detec", value: 20 }

#- node:
#    pkg: "slam_toolbox"
#    exec: "online_sync_launch.py"
#    name: "slam_toolbox"
    
- node:
    pkg: "rviz2"
    exec: "rviz2"
    name: "rviz2"
    args: "-d $(find-pkg-share grp_pibot26)/rviz_config/config_map.rviz"
