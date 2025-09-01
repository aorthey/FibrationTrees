FIBRRT_PRIO="(0.4,0.9,0.4,1.0)"
FIBRRT_DECOMP="(0.3,0.7,0.3,1.0)"
RRT="(1.0,0.7,0.4,1.0)"

FMT="(0.78,0.85,0.94,1.0)"
LBTRRT="(1.0,0.93,0.55,1.0)"
EST="(1.0,0.42,0.41,1.0)"

EXEC=/home/`whoami`/git/ompl_benchmark_plotter/ompl_benchmark_plotter.py
TOPTIONS="--remove-ylabel --no-title --linewidth 20 --only-success-graph --label-fontsize 60 --legend-none --fontsize 70 --planner-color geometric_RRTtask=${RRT} --planner-color geometric_FibrationRrt=${FIBRRT_DECOMP}"
OPTIONS="--remove-ylabel --no-title --linewidth 20 --only-success-graph --label-fontsize 60 --legend-none --fontsize 70  \
--planner-color geometric_FibrationRrt-Decomposition=${FIBRRT_DECOMP}  \
--planner-color geometric_FibrationRrt-Prioritization=${FIBRRT_PRIO} \
--planner-color geometric_FMT=${FMT} \
--planner-color geometric_EST=${EST} \
--planner-color geometric_LBTRRT=${LBTRRT} \
--planner-color geometric_RRT=${RRT}"

###############################################################################
###### 01 Easy Multi Robot
###############################################################################
#python3 ${EXEC} ../data/logs/01_MultiDisks.db ${OPTIONS} --min-time 0.5 --max-time 600
#python3 ${EXEC} ../data/logs/02_DroneCoordination.db ${OPTIONS} --min-time 0.5 --max-time 600
#python3 ${EXEC} ../data/logs/03_MobileNavigation.db ${OPTIONS} --min-time 1.5 --max-time 600
python3 ${EXEC} ../data/logs/04_ReedsSheppSimple.db ${OPTIONS} --min-time 0.01 --max-time 500

###############################################################################
###### 02 Hard Multi Robot
###############################################################################

#python3 ${EXEC} ../data/logs/01_CubeRobots.db ${OPTIONS} --min-time 0.5 --max-time 900
#python3 ${EXEC} ../data/logs/02_DronesInPipe.db ${OPTIONS} --min-time 1.5 --max-time 900
#python3 ${EXEC} ../data/logs/03_MobileRobotForest.db ${OPTIONS} --min-time 1.5 --max-time 1800
#python3 ${EXEC} ../data/logs/04_Warehouse.db ${OPTIONS} --min-time 1.5 --max-time 3600

###############################################################################
###### 03 Task Space Constraints
###############################################################################

#python3 ${EXEC} ../data/logs/01_VerticalMaze.db ${TOPTIONS} --min-time 1.5 --max-time 720
#python3 ${EXEC} ../data/logs/02_MultiRobotVerticalWall.db ${TOPTIONS} --min-time 50 --max-time 7200
#python3 ${EXEC} ../data/logs/03_MobileManipulators.db ${TOPTIONS} --min-time 1.5 --max-time 1800
#python3 ${EXEC} ../data/logs/04_PathVelocityDecomposition.db ${TOPTIONS} --min-time 1.5 --max-time 720
