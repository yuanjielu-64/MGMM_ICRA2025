<p align="center">
  <img width = "100%" src='gdata/mapsAndCar.png' />
  </p>


# Multi-Goal Motion Memory
A novel approach for efficient multi-goal motion planning in complex environments. Our research has been accepted for presentation at IEEE ICRA 2025. Full paper available at: https://arxiv.org/abs/2407.11399 

## Language
This code is written in C++ 

## Requirements
Our experimental evaluation was conducted on a workstation with:

AMD Ryzen 9 5900X CPU (3.7 GHz)
Ubuntu 20.04 operating system
C++ implementation compiled with g++-9.4.0
Neural network models implemented in Python 3.8 with PyTorch 2.2.2

These specifications are recommendations; if you encounter any issues during setup, please contact us for assistance (ylu22@gmu.edu).

## Run Simulations
First, install the Treelite dependency from https://treelite.readthedocs.io/en/latest/

1. Download the treelite
```
git clone https://github.com/dmlc/treelite.git
cd treelite
mkdir build
cd build
cmake ..
cd ..
```

2. Build Multi-Goal Motion Memory
```
cd ..
./build.sh
```

3. Run Experiments
```
python GenerateResults.py --scene SCENE --planner PLANNER --grid GRID_SIZE
```
Parameters:
SCENE: "Random", "Curves", "Maze", and "Storage".
PLANNER: "Dromos" and "Juve". 
GRID_SIZE: 2, 3, 4
In this code, "Dromos" represents the MGMM, while "Juve" represents the baselines

```
python GenerateResults.py --scene Random --planner Dromos --grid 2
```

4. Visualize Simulations
Run specific scenarios with visualization:
```
./bin/Runner GRunMP data/ParamsSceneCurvesForCar.txt ParamsExtraFile data/Instances/CurvesCarNrGrids2/0.txt MGMMPredictLabel data/PredictLabel/ UseMP Dromos UseMap Curves PlannerStatsFile data/Results/DromosCurvesForCar_2.txt UseGrid 2 UseObstacle 0 TopNumber 5
```
Or try different environments:
```
./bin/Runner GRunMP data/ParamsSceneRandomForCar.txt ParamsExtraFile data/Instances/RandomCarNrGrids4/1.txt MGMMPredictLabel data/PredictLabel/ UseMP Dromos UseMap Random PlannerStatsFile data/Results/DromosRandomForCar_4.txt UseGrid 4 UseObstacle 1 TopNumber 5
```
When the visualization appears:
1. Right-click on "Motion Planner"
2. Select "solve repeat"
3. Click "animate solution" to view the path execution

You can modify parameters to test different scenarios:

Change environment: Replace Curves with Random, Maze, or Storage
Change planner: Replace Dromos with Juve
Adjust grid size: Change grid parameter (2, 3, or 4)

For more configuration options, see GenerateResults.py. 

5. Clean Build Files

```
./clean.sh
```

6. Snake Model Configuration
To switch to the snake-like model, modify data/ParamsCar.txt:
Change from:

```
   CarBodyLength 2.00
   CarBodyWidth  0.9
   CarNrTrailers 0
```

to 
```
   CarBodyLength 0.66
   CarBodyWidth  0.5
   CarNrTrailers 3
```

### Contribution
If you would like to contribute to this project, feel free to submit a pull request or open an issue on GitHub.

### License
This project is licensed under the MIT License. See the LICENSE file for details

### Contact
For any questions or support, please contact:
📧 Yuanjie Lu - ylu22@gmu.edu
