SCRIPT_DIR=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )

cd $SCRIPT_DIR/../build
make -j4 FibrationTreesBenchmaker

#ARGS="-d"
ARGS=""
#./FibrationTreesBenchmaker ${ARGS} --filename ../data/benchmarks/prioritization/01/01_MultDisks.yaml
#./FibrationTreesBenchmaker ${ARGS} --filename ../data/benchmarks/prioritization/01/02_DroneCoordination.yaml
#./FibrationTreesBenchmaker ${ARGS} --filename ../data/benchmarks/prioritization/01/03_MobileNavigation.yaml
#./FibrationTreesBenchmaker ${ARGS} --filename ../data/benchmarks/prioritization/01/04_ReedsSheppSimple.yaml
#
#./FibrationTreesBenchmaker ${ARGS} --filename ../data/benchmarks/prioritization/02/01_CubeRobots.yaml
##./FibrationTreesBenchmaker ${ARGS} --filename ../data/benchmarks/prioritization/02/02_DronesInPipe.yaml
#./FibrationTreesBenchmaker ${ARGS} --filename ../data/benchmarks/prioritization/02/03_MobileRobotForest.yaml
#./FibrationTreesBenchmaker ${ARGS} --filename ../data/benchmarks/prioritization/02/04_Warehouse.yaml

#./FibrationTreesBenchmaker ${ARGS} --filename ../data/benchmarks/decomposition/01/01_MultDisks.yaml
#./FibrationTreesBenchmaker ${ARGS} --filename ../data/benchmarks/decomposition/01/02_DroneCoordination.yaml
#./FibrationTreesBenchmaker ${ARGS} --filename ../data/benchmarks/decomposition/01/03_MobileNavigation.yaml
#./FibrationTreesBenchmaker ${ARGS} --filename ../data/benchmarks/decomposition/01/04_ReedsSheppSimple.yaml

# ./FibrationTreesBenchmaker ${ARGS} --filename ../data/benchmarks/decomposition/02/01_CubeRobots.yaml
# ./FibrationTreesBenchmaker ${ARGS} --filename ../data/benchmarks/decomposition/02/02_DronesInPipe.yaml
./FibrationTreesBenchmaker ${ARGS} --filename ../data/benchmarks/decomposition/02/03_MobileRobotForest.yaml
./FibrationTreesBenchmaker ${ARGS} --filename ../data/benchmarks/decomposition/02/04_Warehouse.yaml
