# scp ./robot/src/* ubuntu@192.168.0.107:/home/ubuntu/catkin_ws/src/pub_sub_testing/src/
echo "Checking ~/.bashrc for configuration"
# Remove the files from the .bashrc if they already exist
sed -i '/pub_sub_testing/d' ~/.bashrc
sed -i '/REDIS_URL/d' ~/.bashrc
sed -i '/REDIS_PORT/d' ~/.bashrc
sed -i '/PERCEPT_CHANNEL/d' ~/.bashrc
sed -i '/COMMAND_CHANNEL/d' ~/.bashrc


echo "Adding configuration to ~/.bashrc"
# Add the files
echo 'Generated from ~/catkin_make_ws/src/pub_sub_testing/init.sh\n'
echo export REDIS_URL=localhost >> ~/.bashrc
echo export REDIS_PORT=6379 >> ~/.bashrc
echo export PERCEPT_CHANNEL=/debug/percept >> ~/.bashrc
echo export COMMAND_CHANNEL=/mobile_base/commands/velocity >> ~/.bashrc

source ~/.bashrc

cd ~/catkin_ws/src/pub_sub_testing

echo "Configuring project with catkin"
chmod +x ./src/listener.py
source ~/catkin_ws/devel/setup.bash

# echo "Building ~/catkin_ws workspace"
# cd ~/catkin_ws && catkin build


echo "Building ~/catkin_make_ws workspace"
cd ~/catkin_make_ws/src/pub_sub_testing/src && dos2unix listener.py
cd ~/catkin_make_ws && catkin_make
source ~/catkin_make_ws/devel/setup.bash

# echo "Starting Kobuki Node"
# roslaunch kobuki_node minimal.launch --screen

# echo "Starting Keyop"
# roslaunch kobuki_keyop safe_keyop.launch --screen

# echo "Starting Astra"
# roslaunch astra_launch astra.launch

# echo "Starting Project"
# roslaunch pub_sub_testing pub_sub_testing.launch

echo "Done!"
