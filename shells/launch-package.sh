if [ "$#" -gt 1 ]; then
    echo "Setting ROS_DOMAIN_ID to $2"
    export ROS_DOMAIN_ID=$2
fi

name=$1
source ./install/setup.bash
ros2 launch ${name}_pkg ${name}_pkg_launch_file.launch.py