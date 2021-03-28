#!/bin/bash

BASE_DIR=$(pwd)
cd $BASE_DIR

echo "[info] building project..."
catkin_make
sleep 1

source $BASE_DIR/devel/setup.bash
sleep 1

echo "[info] roscore"
gnome-terminal -- roscore; sleep 1

echo "[info] execute bicycle_model.py"
gnome-terminal -- rosrun cpp_from_python_bicycle bicycle_model; sleep 1

echo "[info] execute control_node.py"
gnome-terminal -- rosrun cpp_from_python_bicycle control_node; sleep 1

echo "[info] execute bicycle_marker.py"
gnome-terminal -- rosrun cpp_from_python_bicycle bicycle_marker; sleep 1

echo "[info] execute viewer.rviz"
gnome-terminal -- rviz -d ./viewer.rviz; sleep 1

echo "[info] execute get_arrow node.py"
gnome-terminal -- rosrun cpp_from_python_bicycle get_arrow; sleep 1
