rosrun cartographer_ros cartographer_pbstream_to_ros_map   -pbstream_filename /home/ubuntu/Robit_SlamNav/carto_ws/map/my_map.pbstream   -map_filestem /home/ubuntu/Robit_SlamNav/carto_ws/map/Office_map   -resolution 0.05

rosservice call /write_state "{filename: '/home/ubuntu/Robit_SlamNav/carto_ws/map/Office_map.pbstream', include_unfinished_submaps: true}"

####mapping ###
roslaunch cartographer_ros livox_2d.launch


######Localization#####

roslaunch cartographer_ros livox_2d_localization.launch


## reinitialization ###
roslaunch cartographer_ros cartographer_guard.launch