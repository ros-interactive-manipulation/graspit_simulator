
export GRASPIT=`rospack find graspit`/graspit_source
export GRASPIT_PLUGIN_DIR=`rospack find dbase_grasp_planner`/lib

alias graspit='`rospack find graspit`/bin/graspit'
alias graspit_plan='`rospack find graspit`/bin/graspit plugin,libdbase_grasp_planning'