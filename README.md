# Vision-Based-Obstacle-Avoidance-and-Goal-Navigation-for-a-Humanoid-Robot

## Overview
A Webots simulation featuring three Nao humanoid robots in a rectangular arena.
The main robot uses camera colour detection and sonar sensors to avoid red obstacle
robots, navigate around static red boxes, and reach a green goal object.

## How to Run
1. Open Webots
2. File → Open World → select `worlds/robot_field.wbt`
3. Press Play

## Project Structure
- `controllers/main_robot/main_robot.py` — Main robot (blue) with 5-priority navigation
- `controllers/red_robot/red_robot.py` — Red wandering robots
- `worlds/robot_field.wbt` — Simulation world file

## Requirements
- Webots R2023b or later
- Python 3
- Nao robot motion files at: C:\Program Files\Webots\projects\robots\softbank\nao\motions\
