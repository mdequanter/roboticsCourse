if [ "$#" -eq 1 ]; then
    echo "Setting ROS_DOMAIN_ID to $1"
    export ROS_DOMAIN_ID=$1
fi
ros2 run turtlebot3_teleop teleop_keyboard

