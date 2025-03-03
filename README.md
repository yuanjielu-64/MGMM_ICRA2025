<p align="center">
  <img width = "100%" src='gdata/mapsAndCar.png' />
  </p>


# Multi-Goal Motion Memory

## Language
This code is written in C++ 

## Requirements

The experimental evaluation uses a workstation with an AMD Ryzen 9 5900X (CPU: 3.7 GHz) running the Ubuntu 20.04 operating system. The code for the motion planning algorithms is developed in C++ and compiled with g++-9.4.0, while the neural network models are implemented in Python 3.8, leveraging the Pytorch 2.2.2 framework.

The requirements above are just suggestions. If you run into any issue, please contact organizers for help (ylu22@gmu.edu).

## Run Simulations
Before we run the simulation to get the results, go https://treelite.readthedocs.io/en/latest/ to download the treelite

1. Download the treelite
```
git clone https://github.com/dmlc/treelite.git
cd treelite
mkdir build
cd build
cmake ..
```

2. Cmake the Multi-Goal motion memory file
```
cd ..
./build.sh
```

3. Run the experiments
```
python GenerateResults.py --scene SCENE --planner PLANNER --grid 2
```
where SCENE can be Random, Curves, Maze, and Storage.  PLANNER can be "Dromos" and "Juve". In this code, "Dromos" represents the MGMM, while "Juve" represents the baselines

```
python GenerateResults.py --scene Random --planner Dromos --grid 2
```

4. If you want to see the simualtion
For example:
```
./bin/Runner GRunMP data/ParamsSceneCurvesForCar.txt ParamsExtraFile data/Instances/CurvesCarNrGrids2/0.txt MGMMPredictLabel data/PredictLabel/ UseMP Dromos UseMap Curves PlannerStatsFile data/Results/DromosCurvesForCar_2.txt UseGrid 2 UseObstacle 0 TopNumber 5
```

```
./bin/Runner GRunMP data/ParamsSceneRandomForCar.txt ParamsExtraFile data/Instances/RandomCarNrGrids4/1.txt MGMMPredictLabel data/PredictLabel/ UseMP Dromos UseMap Random PlannerStatsFile data/Results/DromosRandomForCar_4.txt UseGrid 4 UseObstacle 1 TopNumber 5
```

When you see the scene, right-click "Motion Planner" and click "solve repeat", then you can click animate solution


Change Curves as Random, Maze and Storage
Change Dromos as Juve
Change 2 to 3 or 4

Please see the code in GenerateResults.py 

5. if you want to clean the cmake,

```
./clean.sh
```

### Contribution
If you would like to contribute to this project, feel free to submit a pull request or open an issue on GitHub.

### License
This project is licensed under the MIT License. See the LICENSE file for details

### Contact
For any questions or support, please contact:
📧 Yuanjie Lu - ylu22@gmu.edu