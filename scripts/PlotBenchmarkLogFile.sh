EXEC=/home/`whoami`/git/ompl_benchmark_plotter/ompl_benchmark_plotter.py

#python3 ${EXEC} ../data/logs/CubeRobots.db --legend-separate-file --label-fontsize 60 --fontsize 70 --only-success-graph --min-time 5 --max-time 1800 --show  --no-title --linewidth 12 --remove-ylabel --ignore-planner geometric_FibrationRrt-PrioritizationSmallToLarge --output-file 01_CubeRobots.pdf
#python3 ${EXEC} ../data/logs/DronesInPipe.db --legend-separate-file --label-fontsize 60 --fontsize 70 --only-success-graph --min-time 5 --max-time 1800 --show  --no-title --linewidth 12 --remove-ylabel --output-file 02_DronesInPipe.pdf
#python3 ${EXEC} ../data/logs/MobileRobotForest.db --legend-separate-file --label-fontsize 60 --fontsize 70 --only-success-graph --min-time 50 --max-time 1800 --show  --no-title --linewidth 12 --remove-ylabel --output-file 03_MobileRobotForest.pdf
#python3 ${EXEC} ../data/logs/Warehouse.db --legend-separate-file --label-fontsize 60 --fontsize 70 --only-success-graph --min-time 5 --max-time 1800 --show  --no-title --linewidth 12 --remove-ylabel --output-file 04_Warehouse.pdf
python3 ${EXEC} ../data/logs/VerticalMaze.db --legend-separate-file --label-fontsize 60 --fontsize 70 --only-success-graph --min-time 0.2 --max-time 720 --show  --no-title --linewidth 12 --remove-ylabel --output-file 01_FixedManipulator.pdf
