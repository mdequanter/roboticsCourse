if [ "$#" -eq 1 ]; then
    echo "Setting ROS_DOMAIN_ID to $1"
    export ROS_DOMAIN_ID=$1
fi

ros2 launch turtlebot3_cartographer cartographer.launch.py use_sim_time:=True
