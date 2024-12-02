name=$1
packagename=${name}_pkg
launchfile=${name}_pkg/launch/${name}_pkg_launch_file.launch.py
source ./install/setup.bash
cd src
ros2 pkg create --build-type ament_python ${name}_pkg --dependencies rclpy std_msgs sensor_msgs geometry_msgs
mkdir ${name}_pkg/launch
sed "s/xxx/${name}/g" ./template_launch > ${launchfile}
chmod u+x ${launchfile}
sed "s/xxx/${name}/g" ./template_setup > ${packagename}/setup.py
sed -e "s/xxx/${name}/g" -e "s/yyy/${name^}/g" ./template_script > ${packagename}/${packagename}/${name}.py