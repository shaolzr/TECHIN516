# final_ws

## Project Overview
`final_ws` is a workspace that contains the final implementation of the project. The main code is located in `src/task_controller/task_controller`, with the following structure:

- `total.py/` - The final version of the project code.
- `runrun.py/` - The maze-solving code.It will run maze.py, run,py and maze1.py, which is maze turtlebot forward, kinova move and turtlebot backwards.
- Other directories - Various test codes used during development.
- `put.csv` - A CSV file containing waypoints for the Kinova robotic arm.



## Running the Project

1. Running a Simple Task
```bash
ros2 run task_controller total   # Runs the simple task
ros2 run task_controller runrun  # Runs the maze task
```



## Directory Structure
final_ws/<br>
|<br>
|- src/<br>
|   |- task_controller/<br>
|   |   |- task_controller/<br>
|   |   |   |- total.py            # Final project code<br>
|   |   |   |- runrun.py           # Maze-solving code<br>
|   |   |   |- maze.py             # Maze forward<br>
|   |   |   |- maze1.py            # Maze forward<br>
|   |   |   |- run.py              # kinova move code<br>
|   |   |   |- (other python)/     # Test code<br>
|   |   |   |- put.csv             # Kinova arm waypoints<br>
|   |   |<br>
|   |   |- setup.py                # Python setup file<br>
|   |   |- package.xml             # ROS 2 package info<br>

