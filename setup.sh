#!/usr/bin/env bash

echo "[INFO] Installing Python dependencies..."
pip install -r requirements.txt

echo "[INFO] Make sure Graphviz is installed system-wide:"
echo " - Ubuntu: sudo apt install graphviz"
echo " - macOS: brew install graphviz"
echo " - Windows: https://graphviz.org/download/"

#  python main.py --bt-struct bt_py/real_robot_example.yaml --ip 172.17.201.180
