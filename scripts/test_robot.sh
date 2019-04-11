#!/bin/bash

cd ~/catkin_ws/src/esc_bot/src

echo "Run Unit Test"
pytest -s test_unit_robot.py

echo "Run System Test"
sleep 5
pytest -s test_system_robot.py

echo "Run Robustness Test"
sleep 5
pytest -s test_robust_robot.py