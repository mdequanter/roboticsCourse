
if [ "$#" -eq 1 ]; then
    echo "Setting ROS_DOMAIN_ID to $1"
    export ROS_DOMAIN_ID=$1
fi

ros2 launch turtlebot3_navigation2 navigation2.launch.py use_sim_time:=True map:=$HOME/mapGazeboTurtlebot${ROS_DOMAIN_ID}.yaml