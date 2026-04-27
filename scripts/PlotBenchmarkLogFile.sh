FIBRRT_PRIO="(0.4,0.9,0.4,1.0)"
FIBRRT_DECOMP="(0.3,0.7,0.3,1.0)"
RRT="(1.0,0.7,0.4,1.0)"
QRRT="(1.0,0.7,0.4,1.0)"

# DiscreteRRT1="(1.0,0.7,0.4,1.0)"
# DiscreteRRT5="(1.0,0.7,0.4,1.0)"
# DiscreteRRT10="(1.0,0.7,0.4,1.0)"
# DiscreteRRT50="(1.0,0.7,0.4,1.0)"
# DiscreteRRT1="(1.0,0.75,0.35,1.0)"
# DiscreteRRT5="(1.0,0.65,0.25,1.0)"
# DiscreteRRT10="(1.0,0.50,0.15,1.0)"
# DiscreteRRT50="(0.95,0.30,0.10,1.0)"
DiscreteRRT1="(1.00,0.9,0.4,1.0)"
DiscreteRRT5="(1.00,0.8,0.3,1.0)"
DiscreteRRT10="(1.00,0.7,0.2,1.0)"
DiscreteRRT50="(1.00,0.6,0.1,1.0)"

FMT="(0.78,0.85,0.94,1.0)"
LBTRRT="(1.0,0.93,0.55,1.0)"
EST="(1.0,0.42,0.41,1.0)"

EXEC=/home/`whoami`/git/ompl_benchmark_plotter/ompl_benchmark_plotter.py
TOPTIONS="--remove-ylabel --no-title --linewidth 20 --only-success-graph --label-fontsize 60 --legend-none --fontsize 70 --planner-color geometric_RRTtask=${RRT} --planner-color geometric_FibrationRrt=${FIBRRT_DECOMP}"
MOPTIONS="--remove-ylabel --no-title --linewidth 20 --only-success-graph --label-fontsize 60 --legend-none --fontsize 70 --planner-color geometric_RRT=${RRT} --planner-color geometric_FibrationRrt=${FIBRRT_DECOMP}"
POPTIONS="--remove-ylabel --no-title --linewidth 20 --only-success-graph --label-fontsize 60 --legend-none --fontsize 70 --planner-color geometric_FibrationRrt-Prioritization=${FIBRRT_PRIO} --planner-color geometric_QRRT=${QRRT}"

DOPTIONS="--remove-ylabel --no-title --linewidth 20 --only-success-graph --label-fontsize 60 --legend-separate-file --fontsize 70 \
--planner-color geometric_FibrationRrt-Decomposition=${FIBRRT_DECOMP}
--planner-color geometric_DiscreteRRT-1=${DiscreteRRT1} \
--planner-color geometric_DiscreteRRT-5=${DiscreteRRT5} \
--planner-color geometric_DiscreteRRT-10=${DiscreteRRT10} \
--planner-color geometric_DiscreteRRT-50=${DiscreteRRT50}"

OPTIONS="--remove-ylabel --no-title --linewidth 20 --only-success-graph --label-fontsize 60 --legend-none --fontsize 70  \
--planner-color geometric_FibrationRrt-Decomposition=${FIBRRT_DECOMP}  \
--planner-color geometric_FibrationRrt-Prioritization=${FIBRRT_PRIO} \
--planner-color geometric_FMT=${FMT} \
--planner-color geometric_EST=${EST} \
--planner-color geometric_LBTRRT=${LBTRRT} \
--planner-color geometric_RRT=${RRT}"

###############################################################################
###### 00 Multi Disks
###############################################################################
# python3 ${EXEC} ../data/logs/01_DisksInCrossing.db ${MOPTIONS} --min-time 0.01 --max-time 180
# python3 ${EXEC} ../data/logs/02_DisksOnSquare.db ${MOPTIONS} --min-time 0.01 --max-time 180
# python3 ${EXEC} ../data/logs/03_DisksInTee.db ${MOPTIONS} --min-time 0.01 --max-time 600
# python3 ${EXEC} ../data/logs/04_DisksInMaze.db ${MOPTIONS} --min-time 0.01 --max-time 600

###############################################################################
###### 01 Easy Multi Robot
###############################################################################
#python3 ${EXEC} ../data/logs/01_MultiDisks_old.db ../data/logs/01_MultiDisks.db ${OPTIONS} --min-time 0.5 --max-time 600
#python3 ${EXEC} ../data/logs/01_MultiDisks.db ${OPTIONS} --min-time 0.5 --max-time 600
#python3 ${EXEC} ../data/logs/02_DroneCoordination.db ${OPTIONS} --min-time 0.5 --max-time 600
#python3 ${EXEC} ../data/logs/03_MobileNavigation.db ${OPTIONS} --min-time 1.5 --max-time 600
#python3 ${EXEC} ../data/logs/04_ReedsSheppSimple.db ${OPTIONS} --min-time 1.5 --max-time 600

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

###############################################################################
###### 04 Prioritization
###############################################################################

#python3 ${EXEC} ../data/logs/01_MultiDisks_Prioritization.db ${POPTIONS} --min-time 0.5 --max-time 600
#python3 ${EXEC} ../data/logs/02_DroneCoordination_Prioritization.db ${POPTIONS} --min-time 0.5 --max-time 600
#python3 ${EXEC} ../data/logs/03_MobileNavigation_Prioritization.db ${POPTIONS} --min-time 1.5 --max-time 600
#python3 ${EXEC} ../data/logs/04_ReedsSheppSimple_Prioritization.db ${POPTIONS} --min-time 1.5 --max-time 600
#
#python3 ${EXEC} ../data/logs/01_CubeRobots_Prioritization.db ${POPTIONS} --min-time 0.5 --max-time 900
##python3 ${EXEC} ../data/logs/02_DronesInPipe_Prioritization.db ${POPTIONS} --min-time 1.5 --max-time 900
#python3 ${EXEC} ../data/logs/03_MobileRobotForest_Prioritization.db ${POPTIONS} --min-time 1.5 --max-time 1800
#python3 ${EXEC} ../data/logs/04_Warehouse_Prioritization.db ${POPTIONS} --min-time 1.5 --max-time 3600
#
#cp ../data/logs/01_MultiDisks_Prioritization.pdf ../data/logs/Prioritization/
#cp ../data/logs/02_DroneCoordination_Prioritization.pdf ../data/logs/Prioritization/
#cp ../data/logs/03_MobileNavigation_Prioritization.pdf ../data/logs/Prioritization/
#cp ../data/logs/04_ReedsSheppSimple_Prioritization.pdf ../data/logs/Prioritization/
#cp ../data/logs/01_CubeRobots_Prioritization.pdf ../data/logs/Prioritization/
#cp ../data/logs/02_DronesInPipePrioritization.pdf ../data/logs/Prioritization/02_DronesInPipe_Prioritization.pdf
#cp ../data/logs/03_MobileRobotForest_Prioritization.pdf ../data/logs/Prioritization/
#cp ../data/logs/04_Warehouse_Prioritization.pdf ../data/logs/Prioritization/

###############################################################################
###### 05 Decomposition
###############################################################################
python3 ${EXEC} ../data/logs/01_MultiDisks_Decomposition.db ${DOPTIONS} --min-time 0.5 --max-time 600
python3 ${EXEC} ../data/logs/02_DroneCoordination_Decomposition.db ${DOPTIONS} --min-time 0.5 --max-time 600
python3 ${EXEC} ../data/logs/03_MobileNavigation_Decomposition.db ${DOPTIONS} --min-time 1.5 --max-time 600
python3 ${EXEC} ../data/logs/04_ReedsSheppSimple_Decomposition.db ${DOPTIONS} --min-time 1.5 --max-time 600

python3 ${EXEC} ../data/logs/01_CubeRobots_Decomposition.db ${DOPTIONS} --min-time 0.5 --max-time 900
python3 ${EXEC} ../data/logs/02_DronesInPipe_Decomposition.db ${DOPTIONS} --min-time 1.5 --max-time 900
python3 ${EXEC} ../data/logs/03_MobileRobotForest_Decomposition.db ${DOPTIONS} --min-time 1.5 --max-time 1800
python3 ${EXEC} ../data/logs/04_Warehouse_Decomposition.db ${DOPTIONS} --min-time 1.5 --max-time 3600

cp ../data/logs/01_MultiDisks_Decomposition.pdf ../data/logs/Decomposition/
cp ../data/logs/02_DroneCoordination_Decomposition.pdf ../data/logs/Decomposition/
cp ../data/logs/03_MobileNavigation_Decomposition.pdf ../data/logs/Decomposition/
cp ../data/logs/04_ReedsSheppSimple_Decomposition.pdf ../data/logs/Decomposition/
cp ../data/logs/01_CubeRobots_Decomposition.pdf ../data/logs/Decomposition/
cp ../data/logs/02_DronesInPipe_Decomposition.pdf ../data/logs/Decomposition/
cp ../data/logs/03_MobileRobotForest_Decomposition.pdf ../data/logs/Decomposition/
cp ../data/logs/04_Warehouse_Decomposition.pdf ../data/logs/Decomposition/
