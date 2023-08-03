cd src/match_mobile_robotics
git submodule update --init --recursive
cd submodules/match_path_planning/splined_voronoi/nlopt/
cmake .
make
sudo make install
cd ../../../../../..