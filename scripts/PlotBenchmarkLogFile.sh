EXEC=/home/`whoami`/git/ompl_benchmark_plotter/ompl_benchmark_plotter.py
TOPTIONS="--remove-ylabel --no-title --show --linewidth 20 --only-success-graph --label-fontsize 60 --legend-separate-file --fontsize 70 --planner-color geometric_RRTtask=(0.9,0.1,0.1,1.0) --planner-color geometric_FibrationRrt=(0.3,0.8,0.3,1.0)"
OPTIONS="--remove-ylabel --no-title --show --linewidth 20 --only-success-graph --label-fontsize 60 --legend-separate-file --fontsize 70"

###############################################################################
###### 01 Easy Multi Robot
###############################################################################
#python3 ${EXEC} ../data/logs/VerticalMaze.db --legend-separate-file --label-fontsize 60 --fontsize 70 --only-success-graph --min-time 0.2 --max-time 720 --show  --no-title --linewidth 12 --remove-ylabel --output-file 01_VerticalMaze.pdf
#python3 ${EXEC} ../data/logs/01_MultiDisks.db --legend-separate-file --label-fontsize 60 --fontsize 70 --only-success-graph --min-time 0.2 --show  --no-title --linewidth 12 --remove-ylabel --output-file 01_MultiDisks.pdf
#python3 ${EXEC} ../data/logs/02_DroneCoordination.db --legend-separate-file --label-fontsize 60 --fontsize 70 --only-success-graph --min-time 0.2 --show  --no-title --linewidth 12 --remove-ylabel --output-file 02_DroneCoordination.pdf
#python3 ${EXEC} ../data/logs/03_MobileNavigation.db --legend-separate-file --label-fontsize 60 --fontsize 70 --only-success-graph --min-time 2.0 --show  --no-title --linewidth 12 --remove-ylabel --output-file 03_MobileNavigation.pdf
#python3 ${EXEC} ../data/logs/04_ReedsSheppSimple.db --legend-separate-file --label-fontsize 60 --fontsize 70 --only-success-graph --min-time 0.01 --show  --no-title --linewidth 12 --remove-ylabel --output-file 04_ReedsSheppSimple.pdf
###############################################################################
###### 02 Hard Multi Robot
###############################################################################

#python3 ${EXEC} ../data/logs/CubeRobots.db ${OPTIONS} --output-file 01_CubeRobots.pdf
#python3 ${EXEC} ../data/logs/DronesInPipe.db ${OPTIONS} --output-file 02_DronesInPipe.pdf

#python3 ${EXEC} ../data/logs/DronesInPipe.db --legend-separate-file --label-fontsize 60 --fontsize 70 --only-success-graph --min-time 5 --max-time 1800 --show  --no-title --linewidth 12 --remove-ylabel --output-file 02_DronesInPipe.pdf

python3 ${EXEC} ../data/logs/MobileRobotForest.db --legend-separate-file --label-fontsize 60 --fontsize 70 --only-success-graph --min-time 50 --max-time 1800 --show  --no-title --linewidth 12 --remove-ylabel --output-file 03_MobileRobotForest.pdf

#python3 ${EXEC} ../data/logs/Warehouse.db --legend-separate-file --label-fontsize 60 --fontsize 70 --only-success-graph --min-time 5 --max-time 1800 --show  --no-title --linewidth 12 --remove-ylabel --output-file 04_Warehouse.pdf
###############################################################################
###### 03 Task Space Constraints
###############################################################################

#python3 ${EXEC} ../data/logs/01_VerticalMaze.db ${TOPTIONS} --min-time 1.5 
#python3 ${EXEC} ../data/logs/02_MultiRobotVerticalWall.db ${TOPTIONS} --min-time 50 
#python3 ${EXEC} ../data/logs/03_MobileManipulators.db ${TOPTIONS} --min-time 1.5
#python3 ${EXEC} ../data/logs/04_PathVelocityDecomposition.db ${TOPTIONS} --min-time 1.5

# python3 ${EXEC} ../data/logs/04_PathVelocityDecomposition.db --legend-separate-file --label-fontsize 60 --fontsize 70 --only-success-graph --no-title --linewidth 12 --remove-ylabel --output-file 04_PathVelocityDecomposition.pdf
