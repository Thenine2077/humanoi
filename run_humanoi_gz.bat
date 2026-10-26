@echo off
REM ===================================================================
REM  Humanoi full robot in Gazebo (WSL): mecanum base + pole + head + camera
REM  + two AR4 arms + AmazingHand. Same arm controllers as the real robot, so
REM  MoveIt and the program player work unchanged.
REM  Opens 3 WSL windows: 1) Gazebo + robot  2) MoveIt + RViz  3) program player
REM  Drive the base:  ros2 run humanoi_teleop keyboard_teleop   (in another WSL window)
REM ===================================================================

set WSL_DISTRO=Ubuntu-24.04
set SRC=source /opt/ros/jazzy/setup.bash ^&^& source /mnt/c/Users/User/Desktop/ar4/ar4_ws/install/setup.bash ^&^& source /mnt/c/Users/User/Desktop/Humanoi/ros2_ws/install/setup.bash

echo [1/3] Gazebo + robot ...
start "Humanoi - Gazebo" wsl.exe -d %WSL_DISTRO% -- bash -ic "%SRC% && ros2 launch humanoi_description sim_with_arm.launch.py; exec bash"
echo Waiting 25 seconds for Gazebo to load the robot and start the arm controllers ...
timeout /t 25 /nobreak >nul

echo [2/3] MoveIt + RViz (opens BLANK on purpose - fixes the WSLg mouse freeze) ...
start "Humanoi - MoveIt + RViz" wsl.exe -d %WSL_DISTRO% -- bash -ic "%SRC% && ros2 launch humanoi_description moveit_full.launch.py; exec bash"
timeout /t 3 /nobreak >nul

echo [3/3] Program player (sim time) ...
start "Humanoi - player" wsl.exe -d %WSL_DISTRO% -- bash -ic "%SRC% && ros2 launch ar_program arm_program.launch.py speed:=0.22 use_sim_time:=true; exec bash"

echo.
echo Done. In RViz: File - Recent Configs - /home/user/.rviz2/dual_moveit.rviz
echo   (or File - Open Config - dual_moveit.rviz). Planning Group: left_arm / right_arm / both_arms
echo The simulated camera publishes /camera/color/image_raw and /camera/depth/image_raw
echo   (add an Image display in RViz to see what the robot sees).
echo To stop: close all three windows (or Ctrl+C in each first).
pause
