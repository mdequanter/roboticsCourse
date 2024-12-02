if [ "$#" -gt 1 ]; then
    echo "Setting ROS_DOMAIN_ID to $2"
    export ROS_DOMAIN_ID=$2
fi

command=$1

ros2 topic echo /patrolEvents