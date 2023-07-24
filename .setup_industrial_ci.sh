# sudo apt-get -qq install -y git --no-upgrade --no-install-recommends | grep -E "Setting up"
git config --global --add safe.directory /home/runner/work/match_mobile_robotics/match_mobile_robotics
git submodule update --init --recursive

cd submodules/match_path_planning/splined_voronoi/nlopt/
cmake .
make
sudo make install
cd ../../../../../..
rosdep update
rosdep install --from-paths src --ignore-src --rosdistro $ROS_DISTRO -y
catkin build
source devel/setup.bash
