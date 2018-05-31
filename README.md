# Udacity Home Service Project

Short description of the main project folders:

`udacity_hs` - metapackage with all other packages (`add_markers`, `pick_objects`, `wall_follower`, `hs_project`)

`hs_project` - main package with all launch files, maps, configs and worlds.

`hs_project\scripts` - location of all `*.sh` scripts for testing.

`hs_project\worlds` - world file `mansion_furnished.world` and generated map `map_mansion.pgm`

`hs_project\config` - RViz config

`hs_project\launch` - all supporting launch files with set params of the project

NOTE: Sometimes AMCL node is not starting correctly when executing `home_service.sh` and to fix it I usually close all window and start it again. Without AMCL node & move_base robot model is not appearing in RViz and there is no green particles of particle filter so robot can't move and can't receive/execute goals. (don't know the exact cause of this behavior ...)
