#!/usr/bin/env bash
set -e
source /opt/ros/noetic/setup.bash
python3 -m venv .venv || true
source .venv/bin/activate
pip install -U pip flask
python3 app.py
