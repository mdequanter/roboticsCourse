name=$1
source ./install/setup.bash
colcon build --packages-select ${name}_pkg
