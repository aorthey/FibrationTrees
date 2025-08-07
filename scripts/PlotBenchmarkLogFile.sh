EXEC=/home/`whoami`/git/ompl_benchmark_plotter/ompl_benchmark_plotter.py
#python3 ${EXEC} ../data/logs/VerticalMaze.db --only-success-graph --min-time 0.1 --max-time 720 --show --title "Vertical Maze"
#python3 ${EXEC} ../data/logs/MultiRobotVerticalWall.db --only-success-graph --min-time 1 --max-time 3600 --show --title "Multi Robot Vertical Maze"
python3 ${EXEC} ../data/logs/MobileManipulators.db --only-success-graph --min-time 1 --max-time 1800 --show --title "Mobile Manipulators"
#python3 ${EXEC} ../data/logs/PathVelocityDecomposition.db --only-success-graph --min-time 1 --max-time 720 --show --title "Path Velocity Decomposition"
