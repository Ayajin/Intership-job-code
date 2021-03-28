#!/bin/bash

echo "[info] roscore"
gnome-terminal -- roscore; sleep 1

echo "[info] execute test_tmap.rviz"
gnome-terminal -- rviz -d ./tmap.rviz; sleep 1

echo "[info] execute heretileconvert.py"
gnome-terminal -- rosrun tile_id_encoder heretileconvert.py; sleep 1

echo "[info] execute necessarytile.py"
gnome-terminal -- rosrun tile_id_encoder necessarytile.py; sleep 1

echo "[info] execute simplelistener.py"
gnome-terminal -- rosrun tile_id_encoder simplelistener.py; sleep 1
