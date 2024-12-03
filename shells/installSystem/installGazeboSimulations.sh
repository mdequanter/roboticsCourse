source ~/.bashrc
cd ~/turtlebot3_ws/src/
git clone -b humble-devel https://github.com/ROBOTIS-GIT/turtlebot3_simulations.git
cd ~/turtlebot3_ws
colcon build --symlink-install
echo 'source /usr/share/gazebo/setup.sh' >> ~/.bashrc
