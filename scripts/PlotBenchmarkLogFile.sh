EXEC=/home/`whoami`/git/ompl_benchmark_plotter/ompl_benchmark_plotter.py
#python3 ${EXEC} ../log/Scenario1.db --min-time 0.0001 --max-time 10 --show --title "Scenario 1 MultiDisks"
#python3 ${EXEC} ../log/Scenario2.db --min-time 1 --min-cost 1.2 --show --title "Scenario 2 Vertical Maze"
python3 ${EXEC} ../log/Scenario2.db
#python3 ${EXEC} ../log/Scenario3.db --min-time 1 --show --title "Scenario 3 Multi-Robot Vertical Maze"
#python3 ${EXEC} ../log/Scenario4.db --min-time 10 --min-cost 10 --show --title "Scenario 4 Mobile Manipulators"
#python3 ${EXEC} ../log/Scenario5.db --min-time 10 --show --title "Scenario 5 UAVs"
#python3 ${EXEC} ../log/Scenario6.db --show --title "Scenario 6 Time-Based Planning"
