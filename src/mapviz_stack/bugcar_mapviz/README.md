# Bugcar configuration for mapviz visualization tool
## Installation and basic configuration

Please refer at the [original swri-robotics/mapviz](https://github.com/swri-robotics/mapviz/)

## Demo
Visualize gps/fix by a green-line overlay on the campus

![alt text](https://github.com/tranqkhue/bugcar/blob/v0.2/src/bugcar_mapviz/doc/mapviz_screenshot_1.png)

## Bing Map API key

Bing Map is free for small testing and integrated into Mapviz, which means no harassed installation

**THIS API KEY IS FOR TESTING ONLY! PLEASE DO NOT ABUSE IT**

Bing Map API key is `Avsqyvz6N-_fMWQEW8yR_h2aeA92mes5Th6i3Ch1RRmER6gSyftm36PZZIfKeny_`

## Important

*Mapviz* can **only** be run after *Navsat_transform_node* has been run

## Origin points of Mapviz

Mapviz use */initialize_origin/local_xy_origins* rosparam to set its map origin.
For our purpose, that *origin* is taken from *datum* of *robot_localization* via a Python script
