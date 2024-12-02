if [ "$#" -eq 1 ]; then
    echo "Setting ROS_DOMAIN_ID to $1"
    export ROS_DOMAIN_ID=$1
fi
ros2 run nav2_map_server map_saver_cli -f ~/mapGazeboTurtlebot${ROS_DOMAIN_ID}
